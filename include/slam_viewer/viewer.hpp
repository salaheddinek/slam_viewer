#pragma once
#include "internal_types.hpp"
#include "linalg.hpp"


#include <vector>
#include <string>


namespace Slam_viewer {

//  +--------------------------------------------------------
//  |       Relevent types for Viewer
//  +--------------------------------------------------------
//  | Here goes all the types that are used in viewer.hpp
//  | these type are efined in "internal_types.hpp"
//  +--------------------------------------------------------

struct Position;  /// contains three floats: x, y, z
struct Quaternion;  /// contains four floats: x, y, z, w
struct Camera_pose;  /// contains one position 'p', and one quaternion 'q'
struct Color;  /// contains three uint8_t: r, g, b


//  +--------------------------------------------------------
//  |       The viewer class
//  +--------------------------------------------------------
//  |
//  |
//  +--------------------------------------------------------


class Viewer {
public:
    inline Viewer(){}

    inline void set_resize_factor(const float resize)
    {m_resize = resize;}

    inline void set_resize_factor_for_links(const float resize_for_links)
    {m_resize_for_links = resize_for_links;}

    inline void set_first_camera_color(const Color color)
    {m_first_color = color;}

    inline void set_last_camera_color(const Color color)
    {m_last_color = color;}

    inline void set_cameras_downsample_factor(const int downsample)
    {m_downsample_cameras = downsample;}

    inline void set_links_downsample_factor(const int downsample)
    {m_downsample_links = downsample;}

    void set_cameras_poses(const std::vector<Camera_pose>& cameras_poses)
    {m_cameras_poses = cameras_poses;}

    inline bool write_cameras_trajectory_to_ply_file(const std::string output_path);

    inline float get_resize_factor() const{return m_resize;}

    inline std::vector<std::string> get_errors_if_unsuccessful() const {return m_errors;}

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

    std::vector<std::string> m_errors;

private:

    inline std::vector<size_t> downsample_num_cameras(const int downsample_ratio) const;

    inline bool make_all_cameras();

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

    bool write_data_to_file(const std::string output_path);

};

}

#include "viewer_impl.hpp"
