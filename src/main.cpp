#include <iostream>

#include "slam_viewer/viewer.hpp"



int main() {

    Slam_viewer::Viewer viewer;
    viewer.set_camera_poses_from_file("../data/3_frames.txt");
    viewer.set_resize_factor(0.05f);
    viewer.set_cameras_downsample_factor(40);
    viewer.set_first_camera_color(0, 0, 3000);
    viewer.set_last_camera_color(255, 0, 0);
    viewer.write_cameras_trajectory_to_ply_file("result.ply");

    return 1;
}

