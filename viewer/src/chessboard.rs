use opencv::{
    calib3d::{draw_chessboard_corners, find_chessboard_corners_sb, find_chessboard_corners_sb_with_meta, CALIB_CB_ACCURACY, CALIB_CB_LARGER, CALIB_CB_MARKER}, core::{flip, Mat, Point2f, Size, TermCriteria_EPS, TermCriteria_MAX_ITER, ToInputArray, Vector, ROTATE_180}, highgui::{imshow, poll_key}, hub_prelude::MatTraitConst, imgproc::{corner_sub_pix, cvt_color, resize, COLOR_GRAY2BGR, INTER_CUBIC}
};

use crate::Port;

pub fn calibrate(wf: &[[u8; 98*98]], nf: &[[u8; 98*98]], board_rows: u16, board_cols: u16) {
    let square_length = 1.0; // 4x4 chessboard on my phone is barely above 1 cm
    let w @ h = 98;

    let mut wf_corners_arr = vec![];
    let mut nf_corners_arr = vec![];
    for (wf_image, nf_image) in wf.iter().zip(nf) {
        let wf_corners = get_chessboard_corners(wf_image, Port::Wf, board_rows, board_cols, false);
        let nf_corners = get_chessboard_corners(wf_image, Port::Wf, board_rows, board_cols, false);
        if wf_corners.is_empty() || nf_corners.is_empty() {
            continue;
        }
        wf_corners_arr.push(wf_corners);
        nf_corners_arr.push(nf_corners);
    }
}

/// Returns empty if there were no corners found.
pub fn get_chessboard_corners(image: &[u8; 98*98], port: Port, board_rows: u16, board_cols: u16, show: bool) -> Vector<Point2f> {
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
    let _chessboard_found = find_chessboard_corners_sb(&im, (board_cols, board_rows).into(), &mut corners, CALIB_CB_ACCURACY).unwrap();
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
    corners
}
