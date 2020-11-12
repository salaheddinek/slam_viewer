#include <iostream>

#include "slam_viewer/viewer.hpp"
#include "slam_viewer/linalg.hpp"
#include <Eigen/Dense>

using slm4 = Slam_viewer::linalg::mat<float, 4, 4>;
using slv4 = Slam_viewer::linalg::vec<float, 4>;

namespace sl = Slam_viewer::linalg;
namespace sm = Slam_viewer::Marithmetic;

void make_pose_matrices(std::vector<slm4> & pose_matrices){
//    slm4 pose1_m4 = Slam_viewer::linalg::identity;
//    pose_matrices.push_back(pose1_m4);

    slm4 pose2_m4;
    pose2_m4  = {{1.00000f, 0.00000f, 0.00000f, 0.10000f},
                {0.00000f, 0.70710f, -0.70710f, 0.50000f},
                {0.00000f, 0.70710f, 0.70710f, 0.30000f},
                {0.00000f, 0.00000f, 0.00000f, 1.00000f}};
    pose_matrices.push_back(pose2_m4);

    slm4 pose3_m4;
    pose3_m4  = {{1.00000f, 0.00000f, 0.00000f, 0.50000f},
                {0.00000f, 0.42261f, -0.90630f, 0.80000f},
                {0.00000f, 0.90630f, 0.42261f, 0.90000f},
                {0.00000f, 0.00000f, 0.00000f, 1.00000f}};

    pose_matrices.push_back(pose3_m4);

}

//int main(int argc, char **argv) {
int main() {

    std::vector<slm4> pose_matrices;
    make_pose_matrices(pose_matrices);

    std::vector<Slam_viewer::Camera_pose> poses;
    for(slm4& pose_m: pose_matrices){
        Slam_viewer::Camera_pose pose;
        pose.p.x = pose_m[0][3];
        pose.p.y = pose_m[1][3];
        pose.p.z = pose_m[2][3];

        pose.q = sm::to_quaternion(pose_m);
        poses.push_back(pose);
    }
    Slam_viewer::Viewer viewer;
    viewer.set_cameras_poses(poses);
    viewer.set_resize_factor(0.07f);
    viewer.set_first_camera_color({0, 255, 0});
    viewer.set_first_camera_color({255, 255, 0});
    viewer.write_cameras_trajectory_to_ply_file("nice.ply");

    return 1;
}

