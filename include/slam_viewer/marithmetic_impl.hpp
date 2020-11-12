#pragma once
#include "marithmetic.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <limits>


Slam_viewer::linalg::mat<float, 3, 3> Slam_viewer::Marithmetic::extract_3x3_mat(
        const linalg::mat<float, 4, 4> mat)
{
    linalg::mat<float, 3, 3> res;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            res[i][j] = mat[i][j];
    return res;
}


bool Slam_viewer::Marithmetic::is_pose_matrix(const linalg::mat<float, 4, 4> mat)
{
    float margin = 10 * std::numeric_limits<float>::epsilon();
    for(int i = 0; i < 3; i++)
        if(fabs(mat[3][i]) > margin)
            return false;
    if(fabs(mat[3][3] - 1) > margin)
        return false;
    for(int i = 0; i < 3; i++)
        if(!std::isfinite(mat[i][3]))
            return false;

    linalg::mat<float, 3, 3> rot = extract_3x3_mat(mat);
    return is_rotation_matrix(rot);
}


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

bool Slam_viewer::Marithmetic::is_rotation_matrix(const linalg::mat<float, 3, 3> mat)
{

    float margin = 10 * std::numeric_limits<float>::epsilon();
    linalg::mat<float, 3, 3> I = linalg::mul(mat, linalg::transpose(mat));
    for(int i = 0; i < 3; i++)
        if(fabs(I[i][i] - 1) > margin)
            return false;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            if(fabs(I[i][j]) > margin && i != j)
                return false;

    if(fabs(linalg::determinant(mat) - 1) > margin)
        return false;
    return true;
}

bool Slam_viewer::Marithmetic::is_float3_vector(const linalg::vec<float, 3> vec)
{
    for(int i = 0; i < 3; i++)
        if(!std::isfinite(vec[i]))
            return false;
    return true;
}

bool Slam_viewer::Marithmetic::is_float4_vector(const linalg::vec<float, 4> vec)
{
    for(int i = 0; i < 4; i++)
        if(!std::isfinite(vec[i]))
            return false;
    return true;
}

Slam_viewer::Quaternion Slam_viewer::Marithmetic::to_quaternion(
        const Slam_viewer::linalg::mat<float, 3, 3> mat)
{
    Quaternion q;
    linalg::vec<float, 4> lq = linalg::rotation_quat(linalg::transpose(mat));
    q.x = lq.x; q.y = lq.y; q.z = lq.z; q.w = lq.w;
    return q;
}

Slam_viewer::Quaternion Slam_viewer::Marithmetic::to_quaternion(const linalg::mat<float,4, 4> mat)
{
    linalg::mat<float, 3, 3> m = extract_3x3_mat(mat);
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

    size_t maxl = 0;
    for(size_t i = 0; i < rows; i++){
        for(size_t j = 0 ; j < cols; j++){
            std::stringstream stmp;
            stmp << std::setprecision(6) << mat[i][j];
            std::string tmp = stmp.str();
            maxl = tmp.size() > maxl ? tmp.size(): maxl;
        }
    }
    for(size_t i = 0; i < rows; i++){
        ss << "[ ";
        for(size_t j = 0; j < cols; j++){
            ss << std::setw(maxl) << std::setprecision(6) << mat[i][j];
            if(j < cols - 1)
                ss << ", ";
        }
        ss << " ]\n";
    }

    std::string str = ss.str();
    std::cout << str;
}
