use std::iter::zip;

use ccalib::make_extrinsics;
use nalgebra::{matrix, vector, Const, Dyn, Matrix3, OMatrix, U2};
use opencv::{
    calib3d::{find_circles_grid, CirclesGridFinderParameters, CALIB_CB_CLUSTERING, CALIB_CB_SYMMETRIC_GRID}, core::{Mat, MatTraitConst, MatTraitConstManual, MatTraitManual, Point2f, Ptr, ToInputArray, ToInputOutputArray, Vec3b, Vector, BORDER_CONSTANT, CV_8UC1}, features2d::{Feature2D, SimpleBlobDetector, SimpleBlobDetector_Params}, highgui::{
        destroy_window, get_window_property, imshow, set_window_title, wait_key, WND_PROP_VISIBLE
    }, imgcodecs::imread_def, imgproc::{ellipse, warp_perspective, INTER_LINEAR, LINE_AA}
};

fn imshow_multi<T: ToInputArray>(imgs: &[T], title: &str) {
    let mut i = 0;
    let mut current_title = format!("{title} (Image {} of {})", i+1, imgs.len());
    imshow("imshow_multi", &imgs[0]).unwrap();
    set_window_title("imshow_multi", &current_title).unwrap();
    loop {
        let c = wait_key(100).unwrap();
        // println!("{} {:?}", c, char::from_u32(c as u32));
        if c == b'q'.into()
            || c == 0x1b // esc
            || get_window_property(&"imshow_multi", WND_PROP_VISIBLE).unwrap() == 0.
        {
            break;
        }

        'change_image: {
            // left arrow
            if c == 81 || c == b'h'.into() || c == b','.into() {
                i = (i + imgs.len() - 1) % imgs.len();
            } else if c == 83 || c == b'l'.into() || c == b'.'.into() {
                i = (i + 1) % imgs.len();
            } else {
                break 'change_image;
            }
            imshow(&"imshow_multi", &imgs[i].input_array().unwrap()).unwrap();
            current_title = format!("{title} (Image {} of {})", i+1, imgs.len());
            set_window_title("imshow_multi", &current_title).unwrap();
        }
    }
    destroy_window("imshow_multi").unwrap();
}

fn fill_circle(
    im: &mut impl ToInputOutputArray,
    center: (f64, f64),
    radius: f64,
    color: [f64; 4],
) -> opencv::Result<()> {
    let cx = (center.0 * 1024.0).round() as i32;
    let cy = (center.1 * 1024.0).round() as i32;
    let r = (radius * 1024.0).round() as i32;
    ellipse(
        im,
        (cx, cy).into(),
        (r, r).into(),
        0.0,
        0.0,
        360.0,
        color.into(),
        -1,
        LINE_AA,
        10,
    )
}

fn stroke_circle(
    im: &mut impl ToInputOutputArray,
    center: (f64, f64),
    radius: f64,
    color: [f64; 4],
    thickness: i32,
) -> opencv::Result<()> {
    let cx = (center.0 * 1024.0).round() as i32;
    let cy = (center.1 * 1024.0).round() as i32;
    let r = (radius * 1024.0).round() as i32;
    ellipse(
        im,
        (cx, cy).into(),
        (r, r).into(),
        0.0,
        0.0,
        360.0,
        color.into(),
        thickness,
        LINE_AA,
        10,
    )
}

fn get_circle_centers(im: &impl ToInputArray) -> Vector<Point2f> {
    let mut centers = Vector::<Point2f>::default();

    let mut params = SimpleBlobDetector_Params::default().unwrap();
    params.min_threshold = 50.0;
    params.max_threshold = 150.0;
    params.min_area = 10.0;
    params.max_area = 10000.0;
    params.filter_by_area = true;
    params.filter_by_convexity = true;
    params.min_convexity = 0.87;
    params.min_circularity = 0.1;
    params.filter_by_circularity = true;
    params.min_inertia_ratio = 0.01;
    params.filter_by_inertia = true;

    let mut circle_grid_finder_params = CirclesGridFinderParameters::default().unwrap();
    circle_grid_finder_params.grid_type = opencv::calib3d::CirclesGridFinderParameters_GridType::SYMMETRIC_GRID;
    let simple_blob_detector = SimpleBlobDetector::create(params).unwrap();
    let feature2d_detector: Ptr<Feature2D> = Ptr::from(simple_blob_detector);

    let pattern_was_found = find_circles_grid(
        im,
        (3, 3).into(),
        &mut centers,
        CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING,
        Some(&feature2d_detector),
        circle_grid_finder_params,
    ).unwrap();
    if pattern_was_found {
        centers
    } else {
        panic!("failed to find circle grid");
    }
}

fn overlay_circles(
    dst: &mut Mat,
    circle_centers: &[(i32, i32)],
    radius: i32,
    k: Matrix3<f64>,
    e: Matrix3<f64>,
) {
    let size = dst.size().unwrap();
    let mut im = Mat::new_size_with_default((1000, 1000).into(), CV_8UC1, [0.; 4].into()).unwrap();
    for cc in circle_centers {
        let ccf = (cc.0 as f64, cc.1 as f64);
        stroke_circle(&mut im, ccf, radius as f64, [255.; 4], 6).unwrap();
    }

    let mut cam = Mat::default();
    let mat = (k*e).transpose(); // nalgebra is column major
    let mat = Mat::new_rows_cols_with_data(3, 3, mat.as_slice()).unwrap();
    warp_perspective(&im, &mut cam, &mat, size, INTER_LINEAR, BORDER_CONSTANT, [0.; 4].into()).unwrap();

    let cam_data = cam.data_bytes().unwrap();
    let dst_data = dst.data_typed_mut::<Vec3b>().unwrap();
    for (cam_p, dst_p) in zip(cam_data, dst_data) {
        let c = *cam_p as f32 / 255.;
        dst_p.0[0] = (dst_p.0[0] as f32*(1.-c)) as u8;
        dst_p.0[1] = (dst_p.0[1] as f32*(1.-c)) as u8;
        dst_p.0[2] = (dst_p.0[2] as f32*(1.-c)) as u8 + *cam_p;
    }
}

fn main() {
    // ===== read images =====
    let circle_centers = [
        (140, 140),
        (500, 140),
        (860, 140),
        (140, 500),
        (500, 500),
        (860, 500),
        (140, 860),
        (500, 860),
        (860, 860),
    ];
    let radius = 128;

    let mut imgs = vec![];
    let mut images_points: Vec<OMatrix<f64, U2, Dyn>> = vec![];
    for arg in std::env::args() {
        let im = imread_def(&arg).unwrap();
        let cc = get_circle_centers(&im);
        images_points.push(
            OMatrix::<f64, U2, Dyn>::from_iterator(cc.len(), cc.into_iter().flat_map(|p| [p.x as f64, p.y as f64]))
        );
        imgs.push(im);
    }
    imshow_multi(&imgs, "Input");

    // ===== calibration =====
    let result = ccalib::calibrate(
        &circle_centers.map(|p| vector![p.0 as f64, p.1 as f64, 0.0]),
        &images_points,
        radius.into(),
        400.0,
        250.0,
        250.0,
        [0., 0., 1000.].into(),
    );
    let estimated_extrinsics = result.index((4.., ..)).reshape_generic(Const::<6>, Dyn((result.len()-4)/6));
    for (i, c) in estimated_extrinsics.column_iter().enumerate() {
        println!("e{i}: {:.6?}", c.as_slice());
    }
    let fx = result[0];
    let fy = result[1];
    let cx = result[2];
    let cy = result[3];
    let k = matrix![
        fx, 0., cx;
        0., fy, cy;
        0., 0., 1.;
    ];


    // ===== draw estimated extrinsics =====
    for (e, im) in estimated_extrinsics.column_iter().zip(&mut imgs) {
        let r = e.fixed_view::<3, 1>(0, 0).into_owned();
        let t = e.fixed_view::<3, 1>(3, 0).into_owned();
        let e = make_extrinsics(r, t);
        overlay_circles(im, &circle_centers, radius, k, e);
    }
    imshow_multi(&imgs, "Estimates");
}
