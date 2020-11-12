#pragma once

#include "internal_types.hpp"


namespace Slam_viewer {
namespace Marithmetic {


inline float angle_between_two_vectors(const linalg::vec<float, 3> vec1,
                                       const linalg::vec<float, 3> vec2);

inline linalg::mat<float, 3, 3> to_rot_matrix3(const Quaternion q);

inline linalg::mat<float, 4, 4> to_pose_matrix4(const Camera_pose pose);

inline Quaternion to_quaternion(const linalg::mat<float,3, 3> mat);

inline Quaternion to_quaternion(const linalg::mat<float,4, 4> mat);

template<typename T> void printv(const T vec, const size_t vec_size, const std::string prefix = "");

template<typename T> void printm(const T mat, const size_t rows, const size_t cols, const std::string prefix = "");
}
}

#include "marithmetic_impl.hpp"
