use std::{cell::{Cell, RefCell}, iter::zip, marker::PhantomData};

use argmin::{core::{observers::ObserverMode, Executor, State}, solver::{gaussnewton::GaussNewton, gradientdescent::SteepestDescent, linesearch::{HagerZhangLineSearch, MoreThuenteLineSearch}, trustregion::{CauchyPoint, Dogleg, Steihaug, TrustRegion}}};
use argmin_observer_slog::SlogLogger;
use nalgebra::{allocator::Allocator, dvector, matrix, vector, Const, DVector, DefaultAllocator, Dim, Dyn, Matrix, Matrix3, Matrix4, OMatrix, OVector, RealField, Rotation3, SMatrix, SVector, Scalar, Storage, Unit, Vector, Vector2, Vector3, U1, U3};
use num_dual::{hessian, Dual2SVec64, Dual2Vec, DualNum, DualNumFloat, DualVec};
use num_traits::{One, Zero};

// k is the intrinsic matrix (fx, fy, cx, cy)
// e is the extrinsic matrix [r_1, r_2, t]
// q is the characteristic matrix of the ellipse
pub fn estimate_center<D: DualNum<f64>>(
    k: Matrix3<D>,
    e: Matrix3<D>,
    q_inv: Matrix3<D>,
) -> Vector2<D> {
    let p = k * e.clone() * q_inv * e.transpose() * Vector3::new(D::zero(), D::zero(), D::one());
    p.xy() / p.z.clone()
}

pub fn make_extrinsics<D: DualNum<f64> + RealField>(r: Vector3<D>, t: Vector3<D>) -> Matrix3<D> {
    let r = rotation_from_scaled_axis(r);
    let mut e = Matrix3::identity();
    e.fixed_view_mut::<3, 2>(0, 0)
        .copy_from(&r.matrix().fixed_view::<3, 2>(0, 0));
    e.fixed_view_mut::<3, 1>(0, 2)
        .copy_from(&t);
    e
}

pub fn make_extrinsics_from_mat4<D: DualNum<f64> + RealField>(m: &Matrix4<D>) -> Matrix3<D> {
    let mut e = Matrix3::identity();
    e.fixed_view_mut::<3, 2>(0, 0)
        .copy_from(&m.fixed_view::<3, 2>(0, 0));
    e.fixed_view_mut::<3, 1>(0, 2)
        .copy_from(&m.fixed_view::<3, 1>(0, 3));
    e
}

pub fn reproject<D: DualNum<f64> + RealField>(
    fx: D,
    fy: D,
    cx: D,
    cy: D,
    r: Vector3<D>,
    t: Vector3<D>,
    object_point: Vector3<f64>,
    circle_radius: f64,
) -> Vector2<D> {
    let _0 = || D::zero();
    let _1 = || D::one();
    let k = matrix![
        fx,   _0(),   cx;
        _0(),   fy,   cy;
        _0(), _0(), _1();
    ];
    let e = make_extrinsics(r, t) * matrix![
        circle_radius, 0., object_point.x;
        0., circle_radius, object_point.y;
        0., 0., 1.;
    ].cast();
    let q_inv = Matrix3::from_diagonal(&[1., 1., -1.].into());
    estimate_center(k, e, q_inv.cast())
}

pub fn reprojection_error<D: DualNum<f64> + RealField>(
    fx: D,
    fy: D,
    cx: D,
    cy: D,
    r: Vector3<D>,
    t: Vector3<D>,
    object_point: Vector3<f64>,
    image_point: Vector2<f64>,
    circle_radius: f64,
) -> Vector2<D> {
    let reproj = reproject(fx, fy, cx, cy, r, t, object_point, circle_radius);
    reproj - image_point.cast()
}

pub fn angular_reprojection_error<D: DualNum<f64> + RealField>(
    fx: D,
    fy: D,
    cx: D,
    cy: D,
    r: Vector3<D>,
    t: Vector3<D>,
    object_point: Vector3<f64>,
    image_point: Vector2<f64>,
    circle_radius: f64,
) -> Vector3<D> {
    let reproj = reproject(fx, fy, cx, cy, r, t, object_point, circle_radius);
    reproj.push(1.0.into()).normalize() - image_point.push(1.0).normalize().cast()
}

pub fn calibrate<D: Dim, S: Storage<f64, Const<2>, D>>(
    object_points: &[Vector3<f64>],
    images_points: &[Matrix<f64, Const<2>, D, S>],
    circle_radius: f64,
    initial_focal_length: f64,
    initial_cx: f64,
    initial_cy: f64,
    initial_target_pos: Vector3<f64>, // initial guess for the target's position in camera space
) -> CalibrationResult<f64> {
    for img_p in images_points {
        assert_eq!(object_points.len(), img_p.ncols());
    }

    let cost = |v: OVector<DualVec<f64, f64, Dyn>, Dyn>| {
        let mut residuals = dvector![];
        let [fx, fy, cx, cy, extrinsics @ ..] = v.as_slice() else { unreachable!() };
        for (image_points, e) in zip(images_points, extrinsics.chunks_exact(6)) {
            for (obj_p, img_p) in zip(object_points, image_points.column_iter()) {
                let fx = fx.clone();
                let fy = fy.clone();
                let cx = cx.clone();
                let cy = cy.clone();
                let r = Vector3::new(e[0].clone(), e[1].clone(), e[2].clone());
                let t = Vector3::new(e[3].clone(), e[4].clone(), e[5].clone());
                let [[x, y]] = reprojection_error(fx, fy, cx, cy, r, t, *obj_p, img_p.into(), circle_radius).data.0;
                residuals = residuals.push(x);
                residuals = residuals.push(y);
            }
        }
        residuals
    };
    let problem = Problem {
        f: cost,
        last_param: None.into(),
        last_value: None.into(),
        last_gradient: None.into(),
        last_hessian: None.into(),
    };

    let mut initial_params = CalibrationResult::default();
    initial_params.intrinsics.fx = initial_focal_length;
    initial_params.intrinsics.fy = initial_focal_length;
    initial_params.intrinsics.cx = initial_cx;
    initial_params.intrinsics.cy = initial_cy;
    for _ in images_points {
        initial_params.extrinsics.push(Extrinsic {
            r: [0.0; 3].into(),
            t: initial_target_pos,
        });
    }
    // Steihaug seems better at dealing with bad initial guesses
    // TODO: do something to get a better initial guess
    let trust_region = TrustRegion::new(Steihaug::new().with_max_iters(20));
    let result = Executor::new(problem.clone(), trust_region)
        .configure(|state| state
            .param(initial_params.encode())
            .max_iters(1_000)
            .target_cost(0.0)
        )
        .add_observer(SlogLogger::term(), ObserverMode::Every(200))
        .run()
        .expect("optimization failed");

    // Dogleg seems to converge faster
    let trust_region = TrustRegion::new(Dogleg::new());
    let result = Executor::new(problem.clone(), trust_region)
        .configure(|state| state
            .param(result.state.best_param.unwrap())
            .max_iters(1_000)
            .target_cost(0.0)
        )
        .add_observer(SlogLogger::term(), ObserverMode::Every(100))
        .run()
        .expect("optimization failed");

    // refine with angular error
    // let cost2 = |v: OVector<DualVec<f64, f64, Dyn>, Dyn>| {
    //     let mut residuals = dvector![];
    //     let [fx, fy, cx, cy, extrinsics @ ..] = v.as_slice() else { unreachable!() };
    //     for (image_points, e) in zip(images_points, extrinsics.chunks_exact(6)) {
    //         for (obj_p, img_p) in zip(object_points, image_points.column_iter()) {
    //             let fx = fx.clone();
    //             let fy = fy.clone();
    //             let cx = cx.clone();
    //             let cy = cy.clone();
    //             let r = Vector3::new(e[0].clone(), e[1].clone(), e[2].clone());
    //             let t = Vector3::new(e[3].clone(), e[4].clone(), e[5].clone());
    //             let [[x, y, z]] = angular_reprojection_error(fx, fy, cx, cy, r, t, *obj_p, img_p.into(), circle_radius).data.0;
    //             residuals = residuals.push(x);
    //             residuals = residuals.push(y);
    //             residuals = residuals.push(z);
    //         }
    //     }
    //     residuals
    // };
    // let problem2 = Problem2 {
    //     f: cost2,
    //     last_param: None.into(),
    //     last_value: None.into(),
    //     last_gradient: None.into(),
    //     last_hessian: None.into(),
    // };
    // let result = Executor::new(problem2, TrustRegion::new(Steihaug::new().with_max_iters(20)))
    //     .configure(|state| state
    //         .param(result.state.best_param.unwrap())
    //         .max_iters(100000)
    //         .target_cost(0.0)
    //     )
    //     .add_observer(SlogLogger::term(), ObserverMode::Every(1000))
    //     .run()
    //     .expect("optimization failed");

    let status = result.state.get_termination_status().clone();
    let best_param = result.state.best_param.unwrap();
    let [fx, fy, cx, cy, ..] = best_param.as_slice() else { unreachable!() };
    let cost = result.state.best_cost;
    println!("Converged after {} iterations", result.state.iter);
    println!("Termination status: {status}");
    println!("fx: {fx}");
    println!("fy: {fy}");
    println!("cx: {cx}");
    println!("cy: {cy}");
    println!("cost: {cost}");
    println!("RMSE: {}", f64::sqrt(cost / (object_points.len() * images_points.len()) as f64));
    println!();
    best_param
}

#[derive(Clone)]
struct Problem<G, M, N>
where
    M: Dim,
    N: Dim,
    G: Fn(CalibrationResult<DualVec<f64, f64, N>>) -> OVector<DualVec<f64, f64, N>, M>,
    DefaultAllocator: Allocator<M> + Allocator<N> + Allocator<N, N> + Allocator<nalgebra::Const<1>, N>,
{
    f: G,
    last_param: RefCell<Option<OVector<f64, N>>>,
    last_value: RefCell<Option<f64>>,
    last_gradient: RefCell<Option<OVector<f64, N>>>,
    last_hessian: RefCell<Option<OMatrix<f64, N, N>>>,
}


impl<G, M, N> Problem<G, M, N>
where
    M: Dim,
    N: Dim,
    G: Fn(OVector<DualVec<f64, f64, N>, N>) -> OVector<DualVec<f64, f64, N>, M>,
    DefaultAllocator: Allocator<M> + Allocator<N> + Allocator<M, N> + Allocator<N, M> + Allocator<N, N> + Allocator<U1, N>,
{
    fn evaluate(&self, param: &OVector<f64, N>) {
        let mut last_param = self.last_param.borrow_mut();
        if last_param.as_ref() != Some(param) {
            let (value, jacobian) = num_dual::jacobian(
                |v| (self.f)(CalibrationResult::decode(&v, false)),
                param.clone(),
            );

            let jacobian_t = jacobian.transpose();
            let gradient = 2.0*jacobian_t.clone()*value.clone();
            let hessian = 2.0 * jacobian_t * jacobian;
            *last_param = Some(param.clone());
            *self.last_value.borrow_mut() = Some(value.norm_squared());
            *self.last_gradient.borrow_mut() = Some(gradient);
            *self.last_hessian.borrow_mut() = Some(hessian);
        }
    }
}

impl<G, M, N> argmin::core::CostFunction for Problem<G, M, N>
where
    M: Dim,
    N: Dim,
    G: Fn(OVector<DualVec<f64, f64, N>, N>) -> OVector<DualVec<f64, f64, N>, M>,
    DefaultAllocator: Allocator<M> + Allocator<N> + Allocator<M, N> + Allocator<N, M> + Allocator<N, N> + Allocator<U1, N>,
{
    type Param = OVector<f64, N>;
    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin_math::Error> {
        self.evaluate(param);
        Ok(self.last_value.borrow().clone().unwrap())
    }
}

impl<G, M, N> argmin::core::Gradient for Problem<G, M, N>
where
    M: Dim,
    N: Dim,
    G: Fn(OVector<DualVec<f64, f64, N>, N>) -> OVector<DualVec<f64, f64, N>, M>,
    DefaultAllocator: Allocator<M> + Allocator<N> + Allocator<M, N> + Allocator<N, M> + Allocator<N, N> + Allocator<U1, N>,
{
    type Param = OVector<f64, N>;
    type Gradient = OVector<f64, N>;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, argmin_math::Error> {
        self.evaluate(param);
        Ok(self.last_gradient.borrow().clone().unwrap())
    }
}

impl<G, M, N> argmin::core::Hessian for Problem<G, M, N>
where
    M: Dim,
    N: Dim,
    G: Fn(OVector<DualVec<f64, f64, N>, N>) -> OVector<DualVec<f64, f64, N>, M>,
    DefaultAllocator: Allocator<M> + Allocator<N> + Allocator<M, N> + Allocator<N, M> + Allocator<N, N> + Allocator<U1, N>,
{
    type Param = OVector<f64, N>;
    type Hessian = OMatrix<f64, N, N>;

    fn hessian(&self, param: &Self::Param) -> Result<Self::Hessian, argmin_math::Error> {
        self.evaluate(param);
        Ok(self.last_hessian.borrow().clone().unwrap())
    }
}

/// Custom special case at 0
pub fn rotation_from_scaled_axis<T, SB>(axisangle: Vector<T, U3, SB>) -> Rotation3<T>
where
    T: nalgebra::SimdRealField,
    T::Element: nalgebra::SimdRealField,

    SB: Storage<T, U3>,
{
    let a = axisangle.into_owned();
    let (axis, angle) = Unit::new_and_get(a.clone());
    if angle.is_zero() {
        let _1 = T::one;
        Rotation3::from_matrix_unchecked(matrix![
            _1(), -a.z.clone(),  a.y.clone();
             a.z.clone(), _1(), -a.x.clone();
            -a.y.clone(),  a.x.clone(), _1();
        ])
    } else {
        Rotation3::from_axis_angle(&axis, angle)
    }
}

#[derive(Clone, Default, Debug)]
pub struct CalibrationResult<D: Scalar> {
    pub intrinsics: CameraIntrinsics<D>,
    pub extrinsics: Vec<Extrinsic<D>>,
}

#[derive(Copy, Clone, Default, Debug)]
pub struct CameraIntrinsics<D: Scalar> {
    pub fx: D,
    pub fy: D,
    pub cx: D,
    pub cy: D,
    pub distortion: Option<[D; 5]>, // opencv distortion model (TODO)
}

#[derive(Copy, Clone, Default, Debug)]
pub struct Extrinsic<D: Scalar> {
    r: Vector3<D>,
    t: Vector3<D>,
}

impl<D: Scalar + Zero + One> CameraIntrinsics<D> {
    pub fn matrix(&self) -> Matrix3<D> {
        matrix![
            self.fx.clone(), D::zero(), self.cx.clone();
            D::zero(), self.fy.clone(), self.cy.clone();
            D::zero(), D::zero(), D::one();
        ]
    }
}

impl<D: Scalar> CalibrationResult<D> {
    fn decode<R: Dim, S: Storage<D, R, U1>>(
        v: &Vector<D, R, S>,
        with_distortion: bool,
    ) -> Self {
        let fx = v[0].clone();
        let fy = v[1].clone();
        let cx = v[2].clone();
        let cy = v[3].clone();

        let start;
        let distortion;
        if with_distortion {
            distortion = Some([
                v[4].clone(),
                v[5].clone(),
                v[6].clone(),
                v[7].clone(),
                v[8].clone(),
            ]);
            start = 9;
        } else {
            distortion = None;
            start = 4;
        }
        assert!((v.len() - start) % 6 == 0);
        let mut extrinsics = vec![];
        for i in (start..v.len()).step_by(6) {
            let rx = v[i].clone();
            let ry = v[i+1].clone();
            let rz = v[i+2].clone();
            let tx = v[i+3].clone();
            let ty = v[i+4].clone();
            let tz = v[i+5].clone();
            extrinsics.push(Extrinsic {
                r: vector![rx, ry, rz],
                t: vector![tx, ty, tz],
            });
        }
        Self {
            intrinsics: CameraIntrinsics { fx, fy, cx, cy, distortion },
            extrinsics,
        }
    }

    fn encode(&self) -> DVector<D> {
        let mut values = Vec::with_capacity(
            4 + 6*self.extrinsics.len() + 5*(self.intrinsics.distortion.is_some() as usize)
        );
        values.push(self.intrinsics.fx.clone());
        values.push(self.intrinsics.fy.clone());
        values.push(self.intrinsics.cx.clone());
        values.push(self.intrinsics.cy.clone());
        if let Some(dist) = &self.intrinsics.distortion {
            values.extend(dist.iter().cloned());
        }
        for ex in &self.extrinsics {
            values.extend(ex.r.iter().cloned());
            values.extend(ex.t.iter().cloned());
        }
        values.into()
    }
}
