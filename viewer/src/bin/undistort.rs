use opencv::{calib3d::undistort, core::{no_array, FileStorage, FileStorage_READ, Mat}, highgui::{imshow, wait_key_def}, hub_prelude::{FileNodeTraitConst, FileStorageTraitConst}, imgcodecs::{imread, IMREAD_COLOR}};
use clap::Parser;

#[derive(Parser)]
struct Cli {
    #[arg(short, long)]
    camera_params: String,
    image: String,
}

fn main() {
    let cli = Cli::parse();
    let (camera_matrix, dist_coeffs) = read_camara_params(&cli.camera_params);
    let im = imread(&cli.image, IMREAD_COLOR).unwrap();
    let mut unim = Mat::default();
    undistort(&im, &mut unim, &camera_matrix, &dist_coeffs, &no_array()).unwrap();
    imshow("distorted", &im).unwrap();
    imshow("undistorted", &unim).unwrap();
    wait_key_def().unwrap();
}

fn read_camara_params(path: &str) -> (Mat, Mat) {
    let fs = FileStorage::new_def(path, FileStorage_READ).unwrap();
    let camera_matrix = fs.get("camera_matrix").unwrap().mat().unwrap();
    let dist_coeffs = fs.get("dist_coeffs").unwrap().mat().unwrap();
    (camera_matrix, dist_coeffs)
}
