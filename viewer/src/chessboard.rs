use opencv::{
    calib3d::{calibrate_camera, draw_chessboard_corners, find_chessboard_corners_sb, find_chessboard_corners_sb_with_meta, stereo_calibrate, CALIB_CB_ACCURACY, CALIB_CB_LARGER, CALIB_CB_MARKER, CALIB_CB_NORMALIZE_IMAGE}, core::{flip, no_array, FileStorage, FileStorage_FORMAT_YAML, FileStorage_WRITE, Mat, Point2f, Point3f, Size, TermCriteria, TermCriteria_COUNT, TermCriteria_EPS, TermCriteria_MAX_ITER, ToInputArray, Vector, ROTATE_180}, highgui::{imshow, poll_key}, hub_prelude::{FileStorageTrait, FileStorageTraitConst, MatTraitConst}, imgproc::{corner_sub_pix, cvt_color, resize, COLOR_GRAY2BGR, INTER_CUBIC}
};

use crate::Port;

pub fn my_stereo_calibrate(
    wf: &[[u8; 98 * 98]],
    nf: &[[u8; 98 * 98]],
    board_rows: u16,
    board_cols: u16,
) {
    let square_length = 1.0; // 4x4 chessboard on my phone is barely above 1 cm
    let mut object_points = Vector::<Point3f>::new();
    for y in 0..board_rows {
        for x in 0..board_cols {
            object_points.push(Point3f::new(x as f32, y as f32, 0.0) * square_length);
        }
    }
    let object_points = object_points;

    let mut wf_corners_arr = Vector::<Vector<Point2f>>::new();
    let mut nf_corners_arr = Vector::<Vector<Point2f>>::new();
    for (wf_image, nf_image) in wf.iter().zip(nf) {
        let Some(wf_corners) = get_chessboard_corners(wf_image, Port::Wf, board_rows, board_cols, false) else { continue };
        let Some(nf_corners) = get_chessboard_corners(nf_image, Port::Nf, board_rows, board_cols, false) else { continue };
        wf_corners_arr.push(wf_corners);
        nf_corners_arr.push(nf_corners);
    }
    let [r, t, e, f, rvecs, tvecs]: &mut [Mat; 6] = &mut Default::default();
    let criteria = TermCriteria {
        typ: TermCriteria_EPS + TermCriteria_COUNT,
        max_count: 30,
        epsilon: 1e-6,
    };
    // let reproj_err = stereo_calibrate(&object_points, &wf_corners_arr, &nf_corners_arr, wf_camera_matrix, wf_dist_coeffs, nf_camera_matrix, nf_dist_coeffs, (98, 98).into(), r, t, e, f, flags, criteria).unwrap();
}

pub fn calibrate_single(
    images: &[[u8; 98 * 98]],
    port: Port,
    board_rows: u16,
    board_cols: u16,
) {
    let mut board_points = Vector::<Point3f>::new();
    for y in 0..board_rows {
        for x in 0..board_cols {
            board_points.push(Point3f::new(x as f32, y as f32, 0.0));
        }
    }

    let corners_arr = images.iter().filter_map(|image| {
        get_chessboard_corners(image, port, board_rows, board_cols, false)
    }).collect::<Vector<Vector<Point2f>>>();
    let object_points: Vector<Vector<Point3f>> = std::iter::repeat(board_points).take(corners_arr.len()).collect();

    let mut camera_matrix = Mat::default();
    let mut dist_coeffs = Mat::default();
    let criteria = TermCriteria {
        typ: TermCriteria_EPS + TermCriteria_COUNT,
        max_count: 30,
        epsilon: f64::EPSILON,
    };
    let reproj_err = calibrate_camera(&object_points, &corners_arr, (98, 98).into(), &mut camera_matrix, &mut dist_coeffs, &mut no_array(), &mut no_array(), 0, criteria).unwrap();
    println!("RMS error: {}", reproj_err);

    let filename = match port {
        Port::Nf => "nearfield.yml",
        Port::Wf => "widefield.yml",
    };
    let mut fs = FileStorage::new_def(filename, FileStorage_WRITE | FileStorage_FORMAT_YAML).unwrap();
    if fs.is_opened().unwrap() {
        fs.write_mat("camera_matrix", &camera_matrix).unwrap();
        fs.write_mat("dist_coeffs", &dist_coeffs).unwrap();
        fs.write_f64("rms_error", reproj_err).unwrap();
        fs.release().unwrap();
    } else {
        println!("Failed to open {}", filename);
    }
}

/// Returns None if there were no corners found.
pub fn get_chessboard_corners(image: &[u8; 98*98], port: Port, board_rows: u16, board_cols: u16, show: bool) -> Option<Vector<Point2f>> {
    let board_rows = i32::from(board_rows);
    let board_cols = i32::from(board_cols);

    let tmp = Mat::new_rows_cols_with_data(98, 98, image).unwrap();
    let mut im = Mat::default();
    flip(&tmp, &mut im, 0).unwrap();
    if port == Port::Wf {
        let tmp = im;
        im = Mat::default();
        opencv::core::rotate(&tmp, &mut im, ROTATE_180).unwrap();
    }
    let mut corners = Vector::<Point2f>::default();
    let _chessboard_found = find_chessboard_corners_sb(&im, (board_cols, board_rows).into(), &mut corners, CALIB_CB_ACCURACY | CALIB_CB_NORMALIZE_IMAGE).unwrap();
    // let chessboard_found = find_chessboard_corners_sb_with_meta(
    //     &im,
    //     (board_cols, board_rows).into(),
    //     &mut corners,
    //     CALIB_CB_MARKER + CALIB_CB_LARGER,
    //     &mut meta,
    // )
    // .unwrap();

    if show {
        let mut im_copy = Mat::default();
        cvt_color(&im, &mut im_copy, COLOR_GRAY2BGR, 0).unwrap();
        let tmp = im_copy;
        let mut im_copy = Mat::default();
        resize(&tmp, &mut im_copy, Size::new(512, 512), 0., 0., INTER_CUBIC).unwrap();
        corners.as_mut_slice().iter_mut().for_each(|x| {
            *x *= 512.0 / 98. as f32;
            let half_pixel = (512.0 / 98. as f32) / 2.;
            *x += (half_pixel, half_pixel).into();
        });

        if corners.len() > 0 {
            draw_chessboard_corners(
                &mut im_copy,
                (board_cols, board_rows).into(),
                &corners,
                !corners.is_empty(),
            )
            .unwrap();
        }

        let name = match port {
            Port::Wf => "wf",
            Port::Nf => "nf",
        };
        imshow(name, &im_copy).unwrap();
        poll_key().unwrap();
    }
    if corners.len() > 0 {
        Some(corners)
    } else {
        None
    }
}
