#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
        0, 1, 0,-eye_pos[1],
        0, 0, 1,-eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}


Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float a = rotation_angle / 180.0 * MY_PI;
    Eigen::Matrix4f mr;
    mr << cos(a), -sin(a), 0, 0,
          sin(a), cos(a), 0, 0,
           0,0,1,0,
           0, 0, 0, 1;
    model = mr * model;//绕Z轴旋转给定角度
    return model;
}

Eigen::Matrix4f get_model_matrix_any(Vector3f &axis, float rotation_angle)//绕任意向量轴旋转
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float a = rotation_angle / 180.0 * MY_PI;

   /* float length = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[0] = axis[0] / length;
    axis[1] = axis[1] / length;
    axis[2] = axis[2] / length;*/
    //将axis转化为单位向量
    Eigen::Vector3f axis1 = axis.normalized();

   //罗德里格斯旋转公式  R=I⋅cos(θ)+(1−cos(θ))⋅axis⋅axisT+sin(θ)⋅axis的叉乘矩阵
    Eigen::Matrix3f x;
    x << 0, -axis1[2], axis1[1],
        axis1[2], 0, -axis1[0],
        -axis1[1], axis1[0], 0;

    Eigen::Matrix3f r;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    r = I * cos(a) + (1 - cos(a)) * axis1 * axis1.transpose() + sin(a) * x;

    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
    R.block(0, 0, 3, 3) = r;//从（0,0）插入r矩阵的3*3范围内容

    return R * model;
    
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f PtoO;
    PtoO << zNear, 0, 0, 0,
        0, zNear, 0, 0, 
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
   //对平面进行挤压，使其长宽为近平面；

    //引用git佬：
    //为了使得三角形是正着显示的，这里需要把透视矩阵乘以下面这样的矩阵
   //参考：http://games-cn.org/forums/topic/%e4%bd%9c%e4%b8%9a%e4%b8%89%e7%9a%84%e7%89%9b%e5%80%92%e8%bf%87%e6%9d%a5%e4%ba%86/
    Eigen::Matrix4f Mt(4, 4);
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    PtoO = PtoO*Mt ;

    float FovY = eye_fov / 180.0 * MY_PI;
    float t = zNear * tan(FovY / 2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    Eigen::Matrix4f pos(4, 4);
    pos << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;//先正交平移到原点

    Eigen::Matrix4f scale;
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;//缩放到{-1,1}平方平面


    projection = scale * pos * PtoO * projection;


    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f axis1(8.0f, 3.0f, 2.0f);
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

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix_any(axis1,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
