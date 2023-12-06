#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

int main() {
    // 图像上的像素坐标
    std::vector<cv::Point2f> image_points = { {100, 150}, {300, 150}, {300, 300}, {100, 300} };

    // 对应的三维坐标
    std::vector<cv::Point3f> world_points = { {0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0} };

    // 相机内参矩阵
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500, 0, 600 / 2, 0, 500, 400 / 2, 0, 0, 1);

    // 无畸变系数
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

    // 旋转向量和平移向量
    cv::Mat rotation_vector, translation_vector;

    // PNP位姿解算
    cv::solvePnP(world_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

    // 旋转向量转换为旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);

    // 打印结果
    std::cout << "平移向量：" << translation_vector << std::endl;
    std::cout << "旋转矩阵：" << rotation_matrix << std::endl;

    return 0;
}
