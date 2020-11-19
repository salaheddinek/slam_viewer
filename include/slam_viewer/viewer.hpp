#pragma once
#include "linalg.hpp"

#include <array>
#include <vector>
#include <string>


namespace Slam_viewer {

//  +--------------------------------------------------------
//  |       Relevent types for Viewer
//  +--------------------------------------------------------
//  |
//  | Here goes all the types that are used in viewer.hpp
//  |Quaternion
//  +--------------------------------------------------------

struct Position {
    float x, y ,z;
};
struct Quaternion {
    float x, y, z, w;
    inline Quaternion operator*(const Quaternion& q1) const;
    inline void from_euler_in_degrees(const float rx, const float ry, const float rz);
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


//  +--------------------------------------------------------
//  |       The viewer class
//  +--------------------------------------------------------
//  |
//  |
//  |
//  | PS: This class throws std::runtime_error in case of failure
//  |
//  +--------------------------------------------------------


class Viewer {
public:
    inline Viewer(){}

    //! print all settings used by this class using std::cout
    inline void print_settings() const;

    //! resize factor is used to set the dimensions of the cameras and links between them
    inline void set_resize_factor(const float resize)
    {m_resize = resize;}

    //! link resize factor is used to set the dimensions of the links alone
    inline void set_resize_factor_for_links(const float resize_for_links)
    {m_resize_for_links = resize_for_links;}

    //! set the color of the first camera in RGB, each channel shoud be between 0 and 255
    inline void set_first_camera_color(const int r, const int g, const int b);

    //! set the color of the last camera in RGB, each channel shoud be between 0 and 255
    inline void set_last_camera_color(const int r, const int g, const int b);

    //! set how many cameras will be shown, 0 means no camera will be shown
    inline void set_cameras_downsample_factor(const int downsample)
    {m_downsample_cameras = downsample;}

    //! set how many links will be shown, 0 means no links between cameras will be shown
    inline void set_links_downsample_factor(const int downsample)
    {m_downsample_links = downsample;}

    //! each camera pose determine the orientation and the position of the camera in 3D
    inline void set_cameras_poses(const std::vector<Camera_pose>& cameras_poses)
    {m_cameras_poses = cameras_poses;}

    //! calculate the geometry of the cameras and save the 3D to a .ply file
    inline void write_cameras_trajectory_to_ply_file(const std::string output_path);

    //! if true then the class will print info message
    inline void set_verbose(const bool verbose)
    {m_verbose = verbose;}

    //! get the resize factor of cameras
    inline float get_resize_factor() const{return m_resize;}

    //! ger camera poses from file, each line in the file should be on the form [... p.x p.y p.z q.x q.y q.z q.w]
    inline static std::vector<Camera_pose>
    load_camera_poses_from_file(const std::string poses_file_path);

//  +--------------------------------------------------------
//  |       The private viewer class functions
//  +--------------------------------------------------------
//  |
//  | These are the viewer private function and member variables
//  |
//  +--------------------------------------------------------

private:
    std::vector<Camera_pose> m_cameras_poses;
    std::vector<Point> m_point_cloud;
    std::vector<Triangle> m_vertices;
    Color m_first_color {255, 0, 0};
    Color m_last_color {0, 0, 255};
    std::vector<Color> m_cameras_colors;

    float m_resize {0.04f};
    float m_resize_for_links {0.05f};

    int m_downsample_cameras {1};
    int m_downsample_links {1};
    bool m_verbose {false};

    size_t m_camera_idx {0};
    size_t m_link_idx {0};

private:

    inline void normalize_quaternions();

    inline std::vector<size_t> downsample_num_cameras(const int downsample_ratio) const;

    inline void make_all_cameras();

    inline void make_camera_geometry(const Color color,
                                     const Camera_pose pose);

    inline void set_cameras_colors(const size_t size);


    inline linalg::mat<float, 3, 3> get_rotation_between_two_cam_centers(
            const linalg::vec<float, 3>& cam1,
            const linalg::vec<float, 3>& cam2) const;


    inline void make_one_standard_link(std::vector<std::array<float, 3>>& points,
                                       std::vector<std::array<uint32_t, 3>>& triangles) const;


    inline void make_cameras_link(const Color color1,
                                  const Color color2,
                                  const Camera_pose & pose1,
                                  const Camera_pose & pose2);

    void write_data_to_file(const std::string output_path);

    std::string cam_idx() const;

    std::string link_idx() const;

    uint8_t color_bound(const int c) const;

    void vcout(const std::string msg) const;

    void vcerr(const std::string msg) const;

};

}

#include "viewer_impl.hpp"
