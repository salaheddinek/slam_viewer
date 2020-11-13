#pragma once

#include "viewer.hpp"


namespace Slam_viewer {
namespace Marithmetic {


inline linalg::mat<float, 3, 3> extract_3x3_mat(const linalg::mat<float, 4, 4> mat);

inline float angle_between_two_vectors(const linalg::vec<float, 3> vec1,
                                       const linalg::vec<float, 3> vec2);

inline linalg::mat<float, 3, 3> to_rot_matrix3(const Quaternion q);

inline linalg::mat<float, 4, 4> to_pose_matrix4(const Camera_pose pose);

inline bool is_rotation_matrix(const linalg::mat<float, 3, 3> mat);

inline bool is_pose_matrix(const linalg::mat<float, 4, 4> mat);

inline bool is_float3_vector(const linalg::vec<float, 3> vec);

inline bool is_float4_vector(const linalg::vec<float, 4> vec);

inline bool is_finite(const Camera_pose pose);

inline Quaternion to_quaternion(const linalg::mat<float,3, 3> mat);

inline Quaternion to_quaternion(const linalg::mat<float,4, 4> mat);

inline Quaternion normalize(const Quaternion q);

template<typename T> void printv(const T vec, const size_t vec_size, const std::string prefix = "");

template<typename T> void printm(const T mat, const size_t rows, const size_t cols, const std::string prefix = "");

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6);

}
}

#include "marithmetic_impl.hpp"
