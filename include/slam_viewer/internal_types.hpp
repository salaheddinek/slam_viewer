#pragma once

#include "viewer.hpp"

#include <array>


namespace Slam_viewer {

struct Position {
    float x, y ,z;
};
struct Quaternion {
    float x, y, z, w;
};
struct Camera_pose{
    Position p;
    Quaternion q;
};

struct Color {
    uint8_t r, g, b;
};

struct Triangle {
    uint32_t a, b, c;
};

struct Point {
    float x, y, z;
    Slam_viewer::Color c;
};

//using Vector3 = std::array<float, 3>;

//using Vector4 = std::array<float, 4>;

//using Matrix3 = std::array<std::array<float, 3>, 3>;

//using Matrix4 = std::array<std::array<float, 4>, 4>;

}
