#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen/3.4.0_1/include/eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv/4.9.0_6/include/opencv4/opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
            -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radian = rotation_angle / 180 * MY_PI;
    model << cos(radian), -sin(radian), 0, 0, sin(radian), cos(radian), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    return model;
}


/* 绕任意轴旋转矩阵 */
Eigen::Matrix4f get_axis_model_matrix(float rotation_angle, Eigen::Vector3f axis)
{
    float angle = rotation_angle / 180 * MY_PI;
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
    N << 0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;
    Eigen::Matrix3f rod = std::cos(angle) * Eigen::Matrix3f::Identity() + (1 - std::cos(angle)) * axis * axis.transpose() + std::sin(angle) * N;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    model << rod(0, 0), rod(0, 1), rod(0, 2), 0,
            rod(1, 0), rod(1, 1), rod(1, 2), 0,
            rod(2, 0), rod(2, 1), rod(2, 2), 0,
            0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float tan_half_fov = tan(eye_fov * M_PI / 360.0);
    float t = zNear * tan_half_fov;
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    projection << 2*zNear/(r-l), 0, (r+l)/(r-l), 0,
            0, 2*zNear/(t-b), (t+b)/(t-b), 0,
            0, 0, -(zFar+zNear)/(zFar-zNear), -2*zFar*zNear/(zFar-zNear),
            0, 0, -1, 0;

    return projection;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 10};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    std::vector<Eigen::Vector3f> pos2{{2, 0, -1}, {0, 2, -1}, {-2, 0, -1}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    auto pos_id2 = r.load_positions(pos2);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_axis_model_matrix(angle, {1, 0, 0}));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(90, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        r.draw(pos_id2, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 5;
        }
        else if (key == 'd') {
            angle -= 5;
        } else if (key == 'w') {
            eye_pos.z() -= 1;
        } else if (key == 's') {
            eye_pos.z() += 1;
        }

    }

    return 0;
}
