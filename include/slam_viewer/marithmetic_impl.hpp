#pragma once
#include "marithmetic.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>


float Slam_viewer::Marithmetic::angle_between_two_vectors(const linalg::vec<float, 3> vec1,
                                                          const linalg::vec<float, 3> vec2)
{
    float dot = vec1[0] *vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
    float lenSq1 = vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2];
    float lenSq2 = vec2[0]*vec2[0] + vec2[1]*vec2[1] + vec2[2]*vec2[2];
    float angle = acosf(dot/sqrtf(lenSq1 * lenSq2));
    return angle;
}


inline Slam_viewer::linalg::mat<float, 3, 3> Slam_viewer::Marithmetic::to_rot_matrix3(
        const Slam_viewer::Quaternion q)
{

    linalg::vec<float, 4> lq;
    lq.x = q.x; lq.y = q.y; lq.z = q.z; lq.w = q.w;
    linalg::mat<float, 3, 3> rot = linalg::qmat(lq);
    return linalg::transpose(rot);
//    return rot;
}
inline Slam_viewer::linalg::mat<float, 4, 4> Slam_viewer::Marithmetic::to_pose_matrix4(
        const Slam_viewer::Camera_pose pose)
{
    linalg::mat<float, 3, 3> rot;
    rot = to_rot_matrix3(pose.q);
    linalg::mat<float, 4, 4> res = linalg::identity;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            res[i][j] = rot[i][j];
    res[0][3] = pose.p.x;
    res[1][3] = pose.p.y;
    res[2][3] = pose.p.z;

    return res;
}

inline Slam_viewer::Quaternion Slam_viewer::Marithmetic::to_quaternion(
        const Slam_viewer::linalg::mat<float, 3, 3> mat)
{
    Quaternion q;
    linalg::vec<float, 4> lq = linalg::rotation_quat(linalg::transpose(mat));
    q.x = lq.x; q.y = lq.y; q.z = lq.z; q.w = lq.w;
    return q;
}

Slam_viewer::Quaternion Slam_viewer::Marithmetic::to_quaternion(const linalg::mat<float,4, 4> mat)
{
    linalg::mat<float, 3, 3> m;
    m[0][0] = mat[0][0]; m[0][1] = mat[0][1]; m[0][2] = mat[0][2];
    m[1][0] = mat[1][0]; m[1][1] = mat[1][1]; m[1][2] = mat[1][2];
    m[2][0] = mat[2][0]; m[2][1] = mat[2][1]; m[2][2] = mat[2][2];
    return to_quaternion(m);
}

template<typename T> void Slam_viewer::Marithmetic::printv(
        const T vec, size_t size, const std::string prefix)
{
    std::stringstream ss;
    if(prefix != "")
        ss << prefix << " = ";

    ss << "[ ";
    for(size_t i = 0; i < size; i++){
        ss << std::setprecision(6) << vec[i];
        if(i < size - 1)
            ss << ", ";
    }
    ss << " ]\n";

    std::string str = ss.str();
    std::cout << str;
}

template<typename T> void Slam_viewer::Marithmetic::printm(
        const T mat, const size_t rows, const size_t cols, const std::string prefix)
{
    std::stringstream ss;
    if(prefix != "")
        ss << prefix << " = \n";

    for(size_t i = 0; i < rows; i++){
        ss << "[ ";
        for(size_t j = 0; j < cols; j++){
            ss << std::setw(7) << std::setprecision(6) << mat[i][j];
            if(j < cols - 1)
                ss << ", ";
        }
        ss << " ]\n";
    }

    std::string str = ss.str();
    std::cout << str;
}
