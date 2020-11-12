#pragma once

#include "internal_types.hpp"
#include "viewer.hpp"
#include "marithmetic.hpp"


#include <limits>
#include <algorithm>
#include <fstream>


bool Slam_viewer::Viewer::write_cameras_trajectory_to_ply_file(const std::string output_path)
{
    // clear used member variables
    m_cameras_colors.clear();
    m_errors.clear();
    m_vertices.clear();
    m_point_cloud.clear();
    m_vertices.clear();

    // make the cameras and links geometries
    if(!this->make_all_cameras())
        return false;


    // check if string ends with '.ply', and add it
    std::string path = output_path;
    std::string extention = ".ply";
    if(path.size() >= extention.size()){
        if(path.compare (path.length() - extention.length(), extention.length(), extention) != 0){
            path += extention;
        }
    } else {
        path += extention;
    }
    std::cout << "path: " << path << std::endl;
    // save the result to output file path
    if(!write_data_to_file(path))
        return false;
    return true;
}


std::vector<size_t> Slam_viewer::Viewer::downsample_num_cameras(
        const int downsample_ratio) const
{
    std::vector<size_t> indices;
    if(downsample_ratio <= 0)
        return indices;

    if(downsample_ratio == 1){
        for(uint i = 0; i < m_cameras_poses.size(); i++)
            indices.push_back(i);
        return indices;
    }
    for(size_t i = 0; i < m_cameras_poses.size(); i++)
        if(i % downsample_ratio == 0)
            indices.push_back(i);
    if(indices.back() != m_cameras_poses.size() - 1)
        indices.push_back(m_cameras_poses.size() - 1);
    return indices;
}

bool Slam_viewer::Viewer::make_all_cameras()
{
    if(m_cameras_poses.empty()){
        m_errors.push_back("Slam_viewer: cameras poses array is empty.");
        return false;
    }

    set_cameras_colors(static_cast<uint>(m_cameras_poses.size()));

    std::vector<size_t> cameras_indices = downsample_num_cameras(m_downsample_cameras);

    for(uint i: cameras_indices)
        make_camera_geometry(m_cameras_colors[i], m_cameras_poses[i]);

    std::vector<size_t> links_indices = downsample_num_cameras(m_downsample_links);

    for(size_t i = 1; i < links_indices.size(); i++){
        size_t idx_current = links_indices.at(i);
        size_t idx_previous = links_indices.at(i - 1);
        make_cameras_link(m_cameras_colors.at(idx_previous),
                          m_cameras_colors.at(idx_current),
                          m_cameras_poses.at(idx_previous),
                          m_cameras_poses.at(idx_current));
    }

    return true;
}

void Slam_viewer::Viewer::make_camera_geometry(
        const Color color,
        const Camera_pose pose)
{
    unsigned int bias = m_point_cloud.size();
    linalg::mat<float, 4, 4> pose_m4 = Marithmetic::to_pose_matrix4(pose);

//    Marithmetic::printm(pose_m4, 4, 4, "pose");
    // initial camera vertices positions
    std::vector<linalg::vec<float, 4>> points;
    points = {{0, 0, 0, 1}, {0.75, 0.5, 1, 1}, {-0.75, 0.5, 1, 1},
              {-0.75, -0.5, 1, 1}, {0.75, -0.5, 1, 1},
              {-0.2f, -0.5, 1, 1} , {0.2f, -0.5f, 1, 1}, {0, -0.7f, 1, 1},
              {0, 0, 0.5f, 1}};

    std::vector<Triangle> triangles;
    triangles = {{0, 2, 1}, {0, 1, 4}, {0, 4, 3},
                 {0, 3, 2}, {2, 3, 4}, {1, 2, 4},
                 {6, 5, 7}, {7, 5, 8}, {6, 7, 8}};

    for(uint i = 0; i < 9; i++){
        Point tmp_point;
        tmp_point.c = color;

        // applay camera transformation after resize
        linalg::vec<float, 4> position = points.at(i);
        for(size_t j = 0; j < 3; j++)
            position[j] *= m_resize;

        linalg::vec<float, 4> final_position = linalg::mul(linalg::transpose(pose_m4), position);

//        Marithmetic::printv(position, 4, "ini");
//        Marithmetic::printv(final_position, 4);
        // transform linalg to pcl slam_viewer
        tmp_point.x = final_position[0];
        tmp_point.y = final_position[1];
        tmp_point.z = final_position[2];
        m_point_cloud.push_back(tmp_point);
    }


    for (auto & t : triangles ){
        t.a += bias;
        t.b += bias;
        t.c += bias;

        m_vertices.push_back(t);
    }
}

void Slam_viewer::Viewer::set_cameras_colors(const size_t size)
{
    m_cameras_colors.reserve(size);
    float sizef = static_cast<float>(size);
    // color interpolation
    for (uint i = 0; i < size; i++) {
        Color tmp_color;
        float idx = static_cast<float>(i);
        tmp_color.r = static_cast<uint8_t>(m_first_color.r - (m_first_color.r - m_last_color.r) * idx / sizef);
        tmp_color.g = static_cast<uint8_t>(m_first_color.g - (m_first_color.g - m_last_color.g) * idx / sizef);
        tmp_color.b = static_cast<uint8_t>(m_first_color.b - (m_first_color.b - m_last_color.b) * idx / sizef);
        m_cameras_colors.push_back(tmp_color);
    }
}


Slam_viewer::linalg::mat<float, 3, 3> Slam_viewer::Viewer::get_rotation_between_two_cam_centers(
        const linalg::vec<float, 3>& cam1,
        const linalg::vec<float, 3>& cam2) const
{

    linalg::vec<float, 3> An = cam2 - cam1;
    An = linalg::normalize(An);


    linalg::vec<float, 3> Bn {0, 0, 1};
    linalg::vec<float, 3> new_BN = Bn - An * (An[0] * Bn[0] + An[1] * Bn[1] + An[2] * Bn[2]);

    new_BN  = linalg::normalize(new_BN);

    float angle = Marithmetic::angle_between_two_vectors(An, Bn);

    linalg::mat<float, 3, 3> rot = linalg::identity;
    if(fabs(angle) >= std::numeric_limits<float>::epsilon()){
        linalg::vec<float, 3> axis = linalg::normalize(linalg::cross(An, new_BN));
        linalg::mat<float, 4, 4> rot4 = linalg::rotation_matrix(linalg::rotation_quat(axis, angle));
        for(int ii = 0; ii < 3; ii++)
            for(int jj = 0; jj < 3; jj++)
                rot[ii][jj] = rot4[ii][jj];
    }
//    return linalg::inverse(rot);
    return rot;
}


void Slam_viewer::Viewer::make_one_standard_link(
        std::vector<std::array<float, 3>>& points,
        std::vector<std::array<uint32_t, 3>>& triangles) const
{
    float p = 0.7071067f, n = -p;  //  p = sqrt(2) / 2
    float pp = 0.5773502f, nn = -pp, p20 = pp + 20;  // pp = sqrt(3) / 3

    points = {{0, 0, -1},
              {nn ,nn ,nn}, {pp, nn, nn}, {pp, pp, nn}, {nn, pp, nn},
              {0, -1, 0}, {n, n, 0} , {-1, 0, 0}, {n, p, 0},
              {0, 1, 0}, {p, p, 0}, {1, 0, 0}, {p, n, 0},

              {0, -1, 20}, {n, n, 20} , {-1, 0, 20}, {n, p, 20},
              {0, 1, 20}, {p, p, 20}, {1, 0, 20}, {p, n, 20},
              {nn ,nn ,p20}, {pp, nn, p20}, {pp, pp, p20}, {nn, pp, p20},
              {0, 0, 21}};


    triangles = {{0, 2, 1}, {0, 3, 2}, {0, 4, 3}, {0, 1, 4},

                 {1, 5, 6}, {1, 2, 5}, {5, 2, 12},
                 {2, 11, 12}, {2, 3, 11}, {11, 3, 10},
                 {3, 9, 10}, {3, 4, 9}, {9, 4, 8},
                 {4, 7, 8}, {4, 1, 7}, {7, 1, 6},

                 {5, 13, 6}, {13, 14, 6}, {6, 14, 7}, {14, 15, 7},
                 {7, 15, 8}, {15, 16, 8},  {8, 16, 9}, {16, 17, 9},
                 {9, 17, 10}, {17, 18, 10},  {10, 18, 11}, {18, 19, 11},
                 {11, 19, 12}, {19, 20, 12}, {12, 20, 13}, {13, 5, 12},

                 {19, 22, 20}, {19, 23, 22}, {18, 23, 19},
                 {17, 23, 18}, {17, 24, 23}, {16, 24, 17},
                 {15, 24, 16}, {15, 21, 24}, {14, 21, 15},
                 {13, 21, 14}, {13, 22, 21}, {20, 22, 13},

                 {25, 24, 21}, {25, 21, 22}, {25, 22, 23}, {25, 23, 24}};
}


void Slam_viewer::Viewer::make_cameras_link(
        const Color color1,
        const Color color2,
        const Camera_pose & pose1,
        const Camera_pose & pose2)
{
    uint32_t bias = static_cast<uint32_t>(m_point_cloud.size());

    std::vector<std::array<float, 3>> points;
    std::vector<std::array<uint32_t, 3>> triangles;
    this->make_one_standard_link(points, triangles);

    const uint num_points = 26;

    linalg::mat<float, 4, 4> pose1_m4 = Marithmetic::to_pose_matrix4(pose1);
    linalg::mat<float, 4, 4> pose2_m4 = Marithmetic::to_pose_matrix4(pose2);

    linalg::vec<float, 4> zero = {0, 0, 0, 1};
    linalg::vec<float, 4> cam1 = linalg::mul(linalg::transpose(pose1_m4), zero);
    linalg::vec<float, 4> cam2 = linalg::mul(linalg::transpose(pose2_m4), zero);

    linalg::vec<float, 3> cam1m = {cam1[0], cam1[1], cam1[2]};
    linalg::vec<float, 3> cam2m = {cam2[0], cam2[1], cam2[2]};

    linalg::mat<float, 3, 3> rot = this->get_rotation_between_two_cam_centers(cam1m, cam2m);

    float r = m_resize_for_links * m_resize; // ratio
    for(uint i = 0; i < num_points; i++){
        // decide according to link first or second part
        linalg::vec<float, 4> cam = (i < num_points / 2) ? cam1: cam2;
        Color color = (i < num_points / 2) ? color1: color2;
        float bias = (i < num_points / 2) ? 0: 20;

        // load point
        Point point;
        point.c = color;
        linalg::vec<float, 3> p (points.at(i).at(0), points.at(i).at(1), points.at(i).at(2) - bias);

        // apply transformation on point
        linalg::vec<float, 3> pp = linalg::mul(rot, p);
        point.x = r * pp[0] + cam[0];
        point.y = r * pp[1] + cam[1];
        point.z = r * pp[2] + cam[2];
        m_point_cloud.push_back(point);
    }

    for (auto & triangle : triangles ){
        Triangle v;
        v.a = triangle[0] + bias;
        v.b = triangle[1] + bias;
        v.c = triangle[2] + bias;
        m_vertices.push_back(v);
    }
}

bool Slam_viewer::Viewer::write_data_to_file(const std::string output_path)
{
    std::ofstream strm(output_path);
     if(!strm){
         m_errors.push_back("Error: unable to open file under: " + output_path + "\n");
         return false;
     }
     strm << "ply\n";
     strm << "format ascii 1.0\n";
     strm << "comment Slam Viewer generated\n";
     strm << "element vertex " << m_point_cloud.size()<< "\n";
     strm << "property float x\n";
     strm << "property float y\n";
     strm << "property float z\n";
     strm << "property uchar red\n";
     strm << "property uchar green\n";
     strm << "property uchar blue\n";
     strm << "element face " << m_vertices.size() <<"\n";
     strm << "property list uchar int vertex_indices\n";
     strm << "end_header\n";
     for(auto& p: m_point_cloud)
         strm << p.x << " " << p.y << " " << p.z << " "
              << static_cast<int>(p.c.r) << " " << static_cast<int>(p.c.g)
              << " " << static_cast<int>(p.c.b) << "\n";

     for(auto& t: m_vertices)
         strm << "3 " <<  t.a << " " << t.b << " "  << t.c << "\n";

     strm.close();
     return true;
}
