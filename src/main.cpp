#include <iostream>
#include <cassert>

#include "slam_viewer/viewer.hpp"
#include "cxxopts.hpp"


// used function
void cout_if(const bool verbose, const std::string msg);
void cerr_if(const bool verbose, const std::string msg);

cxxopts::ParseResult args_aparsing(int argc, const char *argv[]);
void apply_angle_correction(const std::vector<float>& correction_angles,
                            const bool verbose,
                            std::vector<Slam_viewer::Camera_pose>& poses);
std::string executable_name();


// main algorithm

int main(int argc, const char *argv[]) {


    cxxopts::ParseResult options = args_aparsing(argc, argv);


    std::vector<Slam_viewer::Camera_pose> cameras;
    {
        Slam_viewer::Camera_pose cam1;
        cam1.p = {0, 0.5f, 1};  // camera position in the form: [x, y, z]
        cam1.q = {0, 0, 0, 1};  // camera orientation in the form of quaternion: [x, y, z, w]
        Slam_viewer::Camera_pose cam2;
        cam2.p = {0, 1, 1};
        cam2.q = {0, 0.7071068f, 0, 0.7071068f};

        std::vector<Slam_viewer::Camera_pose> cameras = {cam1, cam2};


        Slam_viewer::Viewer viewer;
        viewer.set_cameras_poses(cameras);

        viewer.write_cameras_trajectory_to_ply_file("trajectory.ply");
    }
    bool verbose = options.count("verbose");

    cout_if(verbose,"");
    cout_if(verbose," ----- ----- ----- Camera trajectory viewer ----- ----- ----- ");
    cout_if(verbose,"");

    std::vector<Slam_viewer::Camera_pose> poses =
            Slam_viewer::Viewer::load_camera_poses_from_file(
                options["input"].as<std::string>());
    cout_if(verbose, "Successfully loaded poses from file: " + options["input"].as<std::string>());

    apply_angle_correction(options["angle"].as<std::vector<float>>(), verbose, poses);

    Slam_viewer::Viewer viewer;
    viewer.set_verbose(verbose);
    viewer.set_cameras_poses(poses);
    viewer.set_resize_factor(options["resize"].as<float>());
    viewer.set_cameras_downsample_factor(options["subsample"].as<int>());
    viewer.set_links_downsample_factor(options["links"].as<int>());

    std::vector<int> f_color = options["first"].as<std::vector<int>>();
    if(f_color.size() == 3){
        viewer.set_first_camera_color(f_color.at(0), f_color.at(1), f_color.at(2));
    } else {
        cerr_if(verbose, "Warning: Wrong first camera color, use example: --first=<r>,<g>,<b>");
    }

    std::vector<int> l_color = options["last"].as<std::vector<int>>();
    if(l_color.size() == 3){
        viewer.set_last_camera_color(l_color.at(0), l_color.at(1), l_color.at(2));
    } else {
        cerr_if(verbose, "Warning: Wrong last camera color, use example: --last=<r>,<g>,<b>");
    }

    viewer.write_cameras_trajectory_to_ply_file(options["output"].as<std::string>());

    cout_if(verbose,"");
    cout_if(verbose," ----- ----- ----- Viewer terminated ----- ----- ----- ");
    cout_if(verbose,"");

    return 1;
}


cxxopts::ParseResult args_aparsing(int argc, const char *argv[])
{
    std::string app_name = executable_name();
    std::string output_default = std::string(argv[0]) + "_result.ply";
    cxxopts::Options options(app_name,
                             "This program shows the trajectory of camera as .ply file");
    options.allow_unrecognised_options()
            .add_options()
            ("i,input", "Input file path (required)", cxxopts::value<std::string>())
            ("o,output", "Output file path", cxxopts::value<std::string>()
             ->default_value(output_default))
            ("s,subsample", "Subsampling the number of cameras <int>: "
                            "0 means cameras will not be shown.",
             cxxopts::value<int>()->default_value("40"))
            ("k,links", "Subsampling the number of links between cameras"
                                  " <int>: 0 means links will not be shown.",
             cxxopts::value<int>()->default_value("1"))
            ("r,resize", "Resizing camera cones <float>: 0 mean automatic resize",
             cxxopts::value<float>()->default_value("0.04"))
            ("a,angle", "Applied rotation according to x->y->z axis in degrees",
             cxxopts::value<std::vector<float>>()->default_value("0,0,0"))
            ("f,first", "First camera color [r, g, b]",
             cxxopts::value<std::vector<int>>()->default_value("255,0,0"))
            ("l,last", "Last camera color [r, g, b]",
             cxxopts::value<std::vector<int>>()->default_value("0,0,255"))
            ("v,verbose", "Show verbose messages")
            ("h,help", "Print this help")

            ;

    cxxopts::ParseResult options_result = options.parse(argc, argv);

    if (options_result.count("help")){
        std::cout << options.help() << std::endl;
        exit(1);
    }
    options_result.check_for_required_options({"input"});


    return options_result;
}

void apply_angle_correction(const std::vector<float>& correction_angles,
                            const bool verbose,
                            std::vector<Slam_viewer::Camera_pose>& poses)
{
    namespace sl = Slam_viewer::linalg;
    if(correction_angles.size() != 3){
        cerr_if(verbose, "Warning: Wrong format of angle, use example: --angle=<rx>,<ry>,<rz>");
        return;
    }
    std::vector<float> rot = correction_angles;
    if(fabs(rot[0]) + fabs(rot[1]) + fabs(rot[2]) <= (10 * std::numeric_limits<float>::epsilon()))
        return;

    cout_if(verbose, "Applying angle to all poses: [rx=" + std::to_string(rot[0])
            + ", ry=" + std::to_string(rot[1]) + ", rz=" + std::to_string(rot[2]) + "]");


    Slam_viewer::Quaternion correction;
    correction.from_euler_in_degrees(rot[0], rot[1], rot[2]);

//    Slam_viewer::Marithmetic::printv(correction, 4, "correction_quaternion");
    for(auto& pose: poses){
        pose.q = pose.q * correction;
    }

}

std::string executable_name()
{
#if defined(PLATFORM_POSIX) || defined(__linux__) //check defines for your setup

    std::string sp;
    std::ifstream("/proc/self/comm") >> sp;
    return sp;

#elif defined(_WIN32)

    char buf[MAX_PATH];
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    return buf;

#else
    std::string sp = "Slam_viewer";
    assert(false && "unrecognized platform");
    return sp;

#endif
}

void cout_if(const bool verbose, const std::string msg)
{
    if(verbose)
        std::cout << msg << std::endl;
}

void cerr_if(const bool verbose, const std::string msg)
{
    if(verbose)
        std::cerr << msg << std::endl;
}
