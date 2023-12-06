#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

int main() {
    // ͼ���ϵ���������
    std::vector<cv::Point2f> image_points = { {100, 150}, {300, 150}, {300, 300}, {100, 300} };

    // ��Ӧ����ά����
    std::vector<cv::Point3f> world_points = { {0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0} };

    // ����ڲξ���
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500, 0, 600 / 2, 0, 500, 400 / 2, 0, 0, 1);

    // �޻���ϵ��
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

    // ��ת������ƽ������
    cv::Mat rotation_vector, translation_vector;

    // PNPλ�˽���
    cv::solvePnP(world_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

    // ��ת����ת��Ϊ��ת����
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);

    // ��ӡ���
    std::cout << "ƽ��������" << translation_vector << std::endl;
    std::cout << "��ת����" << rotation_matrix << std::endl;

    return 0;
}
