#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <librealsense2/rs.hpp>  // Realsense 库
#include <opencv2/opencv.hpp>    // OpenCV 库，用于图像处理
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <thread>  // For non-blocking input check

namespace fs = std::filesystem;

bool saveImageRequested = false;  // Flag to check if user pressed Enter

// Function to handle non-blocking input
void checkForInput() {
    while (true) {
        char input = std::cin.get();
        if (input == '\n') {
            saveImageRequested = true;
        } else if (input == 'q') {
            ros::shutdown();
            break;
        }
    }
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "tf_to_file_node");
    ros::NodeHandle nh;

    // Create TF listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Open the output file and clear its contents
    std::ofstream outputFile("/home/nvidia/tf_ws/src/tf_listener/robot_pose.txt", std::ios::trunc);
    if (!outputFile.is_open()) {
        ROS_ERROR("Failed to open file for writing");
        return -1;
    }

    // Clear output image folder
    std::string outputDir = "/home/nvidia/tf_ws/src/tf_listener/out_imgs/";
    for (const auto& entry : fs::directory_iterator(outputDir)) {
        fs::remove(entry.path());  // Remove files
    }

    // Initialize Realsense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    // Create window for displaying image
    // cv::namedWindow("Captured Image", cv::WINDOW_AUTOSIZE);

    // Start a thread to listen for user input
    std::thread inputThread(checkForInput);


    // Start the loop to display image stream
    int imageCounter = 0;  // 从 0 开始计数
    while (ros::ok()) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame colorFrame = frames.get_color_frame();
        rs2::video_frame colorVideoFrame = colorFrame.as<rs2::video_frame>();

        // Convert to OpenCV Mat
        int width = colorVideoFrame.get_width();
        int height = colorVideoFrame.get_height();
        cv::Mat colorImage(cv::Size(width, height), CV_8UC3, (void*)colorVideoFrame.get_data(), cv::Mat::AUTO_STEP);

        // Display image
        cv::Mat resizedImage;
        cv::resize(colorImage, resizedImage, cv::Size(640, 480));

        // 显示调整大小后的图像
        cv::imshow("Captured Image", resizedImage);
        cv::waitKey(1);

        // If the user pressed Enter, save the image and pose
        

        // 如果用户按下回车，保存图像和位姿
        if (saveImageRequested) {
            saveImageRequested = false;  // 重置标志

            try {
                // 获取从 base_link 到 link6 的变换
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("base_link", "Link6", ros::Time(0), ros::Duration(1.0));

                // 提取位置（转换为毫米）
                double x = transformStamped.transform.translation.x * 1000.0;
                double y = transformStamped.transform.translation.y * 1000.0;
                double z = transformStamped.transform.translation.z * 1000.0;

                // 提取四元数并转换为欧拉角（RPY，单位为度）
                tf2::Quaternion quat(
                    transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w
                );
                std::cout << "Quaternion: "
                << "x: " << quat.x() << ", "
                << "y: " << quat.y() << ", "
                << "z: " << quat.z() << ", "
                << "w: " << quat.w() << std::endl;
                tf2::Matrix3x3 mat(quat);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);

                // 转换为角度
                roll = roll * 180.0 / M_PI;
                pitch = pitch * 180.0 / M_PI;
                yaw = yaw * 180.0 / M_PI;

                // 将位姿写入输出文件（x, y, z, roll, pitch, yaw）
                outputFile << x << " " << y << " " << z << " "
                        << roll << " " << pitch << " " << yaw << std::endl;
                
                imageCounter++;

                // 控制台输出位姿信息
                ROS_INFO("Position (mm): [x: %.2f, y: %.2f, z: %.2f]", x, y, z);
                ROS_INFO("Orientation (deg): [roll: %.2f, pitch: %.2f, yaw: %.2f]", roll, pitch, yaw);

                // 使用计数器生成文件名
                std::string imagePath = outputDir + std::to_string(imageCounter) + ".png";
                cv::imwrite(imagePath, colorImage);  // 保存图像
                ROS_INFO("Saved image to %s", imagePath.c_str());

                // 增加计数器
                

            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
    }

    // Close file and window
    outputFile.close();
    cv::destroyWindow("Captured Image");

    // Wait for the input thread to finish
    inputThread.join();

    return 0;
}
