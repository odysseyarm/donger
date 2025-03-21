use opencv::{
    calib3d::{calibrate_camera, find_chessboard_corners, find_chessboard_corners_sb, stereo_calibrate, CALIB_FIX_INTRINSIC}, core::{no_array, FileStorage, FileStorage_READ, Mat, Point2f, Point3f, Size, TermCriteria, TermCriteria_COUNT, TermCriteria_EPS, TermCriteria_MAX_ITER, ToInputArray, Vector, _InputArrayTraitConst}, hub_prelude::{FileNodeTraitConst, FileStorageTraitConst}, imgproc::{corner_sub_pix, resize, INTER_CUBIC}
};

use crate::{DeviceUuid, Port};

pub fn read_camera_params(path: &str) -> Option<(Mat, Mat)> {
    let fs = FileStorage::new_def(path, FileStorage_READ).unwrap();
    if !fs.is_opened().unwrap() {
        None
    } else {
        let camera_matrix = fs.get("camera_matrix").unwrap().mat().unwrap();
        let dist_coeffs = fs.get("dist_coeffs").unwrap().mat().unwrap();
        Some((camera_matrix, dist_coeffs))
    }
}

pub fn my_stereo_calibrate(
    wf: &[Vec<u8>],
    nf: &[Vec<u8>],
    resolution: (u16, u16),
    object_resolution: (u32, u32),
    board_rows: u16,
    board_cols: u16,
    wf_image_files: &Vector<String>,
    nf_image_files: &Vector<String>,
    device_uuid: DeviceUuid,
) {
    let Some((nf_camera_matrix, nf_dist_coeffs)) = &mut read_camera_params(&format!("calibrations/{device_uuid}/nearfield.json")) else {
        println!("Couldn't get nearfield intrinsics");
        return;
    };
    let Some((wf_camera_matrix, wf_dist_coeffs)) = &mut read_camera_params(&format!("calibrations/{device_uuid}/widefield.json")) else {
        println!("Couldn't get widefield intrinsics");
        return;
    };

    let square_length = 0.025; // 25 mm
    let mut object_points = Vector::<Point3f>::new();
    for y in 0..board_rows {
        for x in 0..board_cols {
            object_points.push(Point3f::new(x as f32, y as f32, 0.0) * square_length);
        }
    }
    let object_points = object_points;

    let mut object_points_arr = Vector::<Vector<Point3f>>::new();
    let mut wf_corners_arr = Vector::<Vector<Point2f>>::new();
    let mut nf_corners_arr = Vector::<Vector<Point2f>>::new();
    for (wf_image, nf_image) in wf.iter().zip(nf) {
        let Some(wf_corners) =
            get_chessboard_corners(wf_image, Port::Wf, resolution, object_resolution, board_rows, board_cols, false)
        else {
            continue;
        };
        let Some(nf_corners) =
            get_chessboard_corners(nf_image, Port::Nf, resolution, object_resolution, board_rows, board_cols, false)
        else {
            continue;
        };
        wf_corners_arr.push(wf_corners);
        nf_corners_arr.push(nf_corners);
        object_points_arr.push(object_points.clone());
    }
    let mut r = Mat::default();
    let mut t = Mat::default();
    let criteria = TermCriteria {
        typ: TermCriteria_EPS + TermCriteria_COUNT,
        max_count: 30,
        epsilon: 1e-6,
    };
    let reproj_err = stereo_calibrate(
        &object_points_arr,
        &wf_corners_arr,
        &nf_corners_arr,
        wf_camera_matrix,
        wf_dist_coeffs,
        nf_camera_matrix,
        nf_dist_coeffs,
        (object_resolution.0 as i32, object_resolution.1 as i32).into(),
        &mut r,
        &mut t,
        &mut no_array(),
        &mut no_array(),
        CALIB_FIX_INTRINSIC,
        criteria,
    )
    .unwrap();
    println!("RMS error: {}", reproj_err);

    super::save_stereo_calibration(
        "chessboard",
        r,
        t,
        reproj_err,
        wf_image_files,
        nf_image_files,
        device_uuid,
    );
}

pub fn calibrate_single(
    images: &[Vec<u8>],
    image_files: &Vector<String>,
    port: Port,
    resolution: (u16, u16),
    object_resolution: (u32, u32),
    board_rows: u16,
    board_cols: u16,
    device_uuid: DeviceUuid,
) {
    let mut board_points = Vector::<Point3f>::new();
    for y in 0..board_rows {
        for x in 0..board_cols {
            board_points.push(Point3f::new(x as f32, y as f32, 0.0));
        }
    }

    let corners_arr = images.iter().filter_map(|image| {
        get_chessboard_corners(image, port, resolution, object_resolution, board_rows, board_cols, false)
    }).collect::<Vector<Vector<Point2f>>>();
    let object_points: Vector<Vector<Point3f>> = std::iter::repeat(board_points).take(corners_arr.len()).collect();

    let mut camera_matrix = Mat::default();
    let mut dist_coeffs = Mat::default();
    let criteria = TermCriteria {
        typ: TermCriteria_EPS + TermCriteria_COUNT,
        max_count: 30,
        epsilon: f64::EPSILON,
    };
    let reproj_err = calibrate_camera(
        &object_points,
        &corners_arr,
        (object_resolution.0 as i32, object_resolution.1 as i32).into(),
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut no_array(),
        &mut no_array(),
        0,
        criteria,
    ).unwrap();
    println!("RMS error: {}", reproj_err);

    super::save_single_calibration(
        port,
        "chessboard",
        &camera_matrix,
        &dist_coeffs,
        reproj_err,
        image_files,
        device_uuid,
    );
}

/// Returns None if there were no corners found.
pub fn get_chessboard_corners(
    image: &[u8],
    port: Port,
    resolution: (u16, u16),
    object_resolution: (u32, u32),
    board_rows: u16,
    board_cols: u16,
    show: bool,
) -> Option<Vector<Point2f>> {
    let im = Mat::new_rows_cols_with_data(resolution.1.into(), resolution.0.into(), image).unwrap();
    get_chessboard_corners_cv(&im, port, object_resolution, board_rows, board_cols, show)
}

pub fn get_chessboard_corners_cv(
    image: &impl ToInputArray,
    _port: Port,
    object_resolution: (u32, u32),
    board_rows: u16,
    board_cols: u16,
    _show: bool,
) -> Option<Vector<Point2f>> {
    let board_rows = i32::from(board_rows);
    let board_cols = i32::from(board_cols);

    let mut corners = Vector::<Point2f>::default();
    let use_sb = true;
    let mut im_upscaled = Mat::default();
    // super resolution scale
    let ss;
    if image.input_array().unwrap().size_def().unwrap() == (98, 98).into() {
        ss = 4.;
        resize(image, &mut im_upscaled, Size::new(0, 0), ss, ss, INTER_CUBIC).unwrap();
    } else {
        ss = 1.;
        im_upscaled = image.input_array().unwrap().get_mat_def().unwrap();
    }
    if use_sb {
        let _chessboard_found = find_chessboard_corners_sb(&im_upscaled, (board_cols, board_rows).into(), &mut corners, 0).unwrap();
    } else {
        let _chessboard_found = find_chessboard_corners(&im_upscaled, (board_cols, board_rows).into(), &mut corners, 0).unwrap();
        if corners.len() > 0 {
            corner_sub_pix(&im_upscaled, &mut corners, (3, 3).into(), (-1, -1).into(), opencv::core::TermCriteria {
                typ: TermCriteria_EPS + TermCriteria_MAX_ITER,
                max_count: 30,
                epsilon: 0.001,
            }).unwrap();
        }
    }
    // scale corners back down
    corners.as_mut_slice().iter_mut().for_each(|x| {
        x.x = (x.x + 0.5) / ss as f32 - 0.5;
        x.y = (x.y + 0.5) / ss as f32 - 0.5;
    });
    // let chessboard_found = find_chessboard_corners_sb_with_meta(
    //     &im,
    //     (board_cols, board_rows).into(),
    //     &mut corners,
    //     CALIB_CB_MARKER + CALIB_CB_LARGER,
    //     &mut meta,
    // )
    // .unwrap();

    // if show {
    //     let mut im_copy = Mat::default();
    //     cvt_color_def(image, &mut im_copy, COLOR_GRAY2BGR).unwrap();
    //     let tmp = im_copy;
    //     let mut im_copy = Mat::default();
    //     resize(&tmp, &mut im_copy, Size::new(512, 512), 0., 0., INTER_CUBIC).unwrap();
    //     corners.as_mut_slice().iter_mut().for_each(|x| {
    //         x.x = (x.x + 0.5) * 512.0 / 98.0 - 0.5;
    //         x.y = (x.y + 0.5) * 512.0 / 98.0 - 0.5;
    //     });
    //
    //     if corners.len() > 0 {
    //         draw_chessboard_corners(
    //             &mut im_copy,
    //             (board_cols, board_rows).into(),
    //             &corners,
    //             !corners.is_empty(),
    //         )
    //         .unwrap();
    //     }
    //
    //     let name = match port {
    //         Port::Wf => "wf",
    //         Port::Nf => "nf",
    //     };
    //     imshow(name, &im_copy).unwrap();
    //     poll_key().unwrap();
    // }
    if corners.len() > 0 {
        let size = image.input_array().unwrap().size(-1).unwrap();
        for corner in corners.as_mut_slice() {
            corner.x *= (object_resolution.0 - 1) as f32 / (size.width - 1) as f32;
            corner.y *= (object_resolution.1 - 1) as f32 / (size.height - 1) as f32;
        }
        Some(corners)
    } else {
        None
    }
}
