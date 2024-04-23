use opencv::{
    calib3d::{draw_chessboard_corners, find_chessboard_corners_sb}, core::{flip, Mat, Point2f, Size, Vector, ROTATE_180}, highgui::{imshow, poll_key}, imgproc::{cvt_color, resize, COLOR_GRAY2BGR, INTER_CUBIC}
};

pub fn process_image(name: &str, image: &[u8], w: i32, h: i32) {
    let im = Mat::new_rows_cols_with_data(h as i32, w as i32, image).unwrap();
    let squares_x = 4;
    let squares_y = 4;
    let square_length = 1.0;

    let tmp = im;
    let mut im = Mat::default();
    flip(&tmp, &mut im, 0).unwrap();
    if name == "wf" {
        let tmp = im;
        im = Mat::default();
        opencv::core::rotate(&tmp, &mut im, ROTATE_180).unwrap();
    }
    let mut corners = Vector::<Point2f>::default();
    // TODO experiment with flags
    let chessboard_found = find_chessboard_corners_sb(&im, (squares_x, squares_y).into(), &mut corners, 0).unwrap();
    // if charuco_corners.len() > 0 {
    //     corner_sub_pix(&resized, &mut charuco_corners, (2, 2).into(), (-1, -1).into(), opencv::core::TermCriteria {
    //         typ: TermCriteria_EPS + TermCriteria_MAX_ITER,
    //         max_count: 30,
    //         epsilon: 0.001,
    //     }).unwrap();
    // }

    let mut im_copy = Mat::default();
    cvt_color(&im, &mut im_copy, COLOR_GRAY2BGR, 0).unwrap();
    let tmp = im_copy;
    let mut im_copy = Mat::default();
    resize(&tmp, &mut im_copy, Size::new(512, 512), 0., 0., INTER_CUBIC).unwrap();
    corners.as_mut_slice().iter_mut().for_each(|x| *x *= 512.0 / w as f32);

    if corners.len() > 0 {
        draw_chessboard_corners(&mut im_copy, (squares_x, squares_y).into(), &corners, chessboard_found).unwrap();
    }

    imshow(name, &im_copy).unwrap();
    poll_key().unwrap();
}
