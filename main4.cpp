// C++
#include <iostream>
#include <chrono>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Kinect DK
#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>
#include <k4arecord/playback.hpp>
using namespace std;


int main() {

    //写入txt文件流
    ofstream rgb_out;
    ofstream d_out;
    ofstream ir_out;

    rgb_out.open("./rgb.txt");
    d_out.open("./depth.txt");
    ir_out.open("./ir.txt");

    rgb_out << "#  color images" << endl;
    rgb_out << "#  file: rgbd_dataset" << endl;
    rgb_out << "#  timestamp" << "    " << "filename" << endl;

    d_out << "#  depth images" << endl;
    d_out << "#  file: rgbd_dataset" << endl;
    d_out << "#  timestamp" << "    " << "filename" << endl;

    ir_out << "#  ir images" << endl;
    ir_out << "#  file: rgbd_dataset" << endl;
    ir_out << "#  timestamp" << "    " << "filename" << endl;

    rgb_out << flush;
    d_out << flush;


    // 从设备获取捕获
    k4a::image rgbImage;
    k4a::image depthImage;
    k4a::image irImage;
    k4a::image transformed_depthImage;
    k4a::image transformed_colorImage;

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_irImage;
    cv::Mat cv_irImage_8U;

    k4a::playback handle = k4a::playback::open("a.mkv");
    k4a::capture capture;
    k4a::image transformed_depth_image;
    k4a::calibration k4aCalibration;
    k4a::transformation transformation;
    int no_frames{};

    int num = 1;

    k4aCalibration = handle.get_calibration();
    transformation = k4a::transformation(k4aCalibration);
    handle.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
    std::chrono::microseconds current_time(1000);
    
    handle.seek_timestamp(current_time, K4A_PLAYBACK_SEEK_BEGIN);



    while (handle.get_next_capture(&capture))
    {
        //Preocess Capture
        no_frames += 1;

        depthImage = capture.get_depth_image();
        if (!depthImage)
        {
            std::cout << "No depth_image!" << std::endl;
        }
        rgbImage = capture.get_color_image();


        k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
 
        //transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
        transformed_colorImage = k4aTransformation.color_image_to_depth_camera(depthImage, rgbImage);
     
        cv_rgbImage_with_alpha = cv::Mat(transformed_colorImage.get_height_pixels(), transformed_colorImage.get_width_pixels(), CV_8UC4,
            (void*)transformed_colorImage.get_buffer());

                    //cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_with_alpha, cv::COLOR_BGRA2BGR);
        cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
        cv_depth = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U,
            (void*)depthImage.get_buffer(), static_cast<size_t>(depthImage.get_stride_bytes()));
        cv_depth.convertTo(cv_depth, CV_16U, 10);

        // save image
        double time_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
            rgbImage.get_device_timestamp()).count());

        std::string filename_rgb = std::to_string(time_ / 1000000) + ".png";
        //            double time_d = static_cast<double >(std::chrono::duration_cast<std::chrono::microseconds>(
        //                    depthImage.get_device_timestamp()).count());

        std::string filename_d = std::to_string(time_ / 1000000) + ".png";

        //            double time_ir = static_cast<double >(std::chrono::duration_cast<std::chrono::microseconds>(
        //                    irImage.get_device_timestamp()).count());
        std::string filename_ir = std::to_string(time_ / 1000000) + ".png";
        imwrite("./rgb/" + filename_rgb, cv_rgbImage_no_alpha);
        imwrite("./depth/" + filename_d, cv_depth);
        //             imwrite("./depth/"+filename_d,cv_depth_8U);
        //            imwrite("./ir/"+filename_ir, cv_irImage_8U);

        std::cout << num << " succeed!" << endl;
        num++; 


        //写入depth.txt,rgb.txt文件
        rgb_out << std::to_string(time_ / 1000000) << "    " << "rgb/" << filename_rgb << endl;
        d_out << std::to_string(time_ / 1000000) << "    " << "depth/" << filename_d << endl;
        ir_out << std::to_string(time_ / 1000000) << "    " << "ir/" << filename_ir << endl;

        rgb_out << flush;
        d_out << flush;
        ir_out << flush;
        cv_rgbImage_with_alpha.release();
        cv_rgbImage_no_alpha.release();
        cv_depth.release();
        cv_depth_8U.release();
        cv_irImage.release();
        cv_irImage_8U.release();

        capture.reset();
    }

    cv::destroyAllWindows();
    rgb_out << flush;
    d_out << flush;
    ir_out << flush;
    rgb_out.close();
    d_out.close();
    ir_out.close();

    // 释放，关闭设备
    rgbImage.reset();
    depthImage.reset();
    irImage.reset();
    capture.reset();

    return 1;

    //number of frames
    cout << no_frames << endl;
}