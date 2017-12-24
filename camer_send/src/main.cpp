

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "withrobot_camera.hpp"
 #include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

int main(int argc, char* argv[])
{

    sensor_msgs::ImagePtr img_msg; 
    ros::init(argc, argv, "april_pic");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("april_pic/image", 1);
    std_msgs::Header header;
    header.frame_id="mono8";
    const char* devPath = "/dev/video0";
    Withrobot::Camera camera(devPath);
    camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, 60);
    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();
    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");
    if (!camera.start()) {
        perror("Failed to start.");
        exit(0);
    }
    std::string windowName = camName + " " + camSerialNumber;
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC2);
    cv::Mat gray(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat con = (cv::Mat_<double>(4, 1) << -0.433630707186419, 0.219013828295360, 0.0, 0.0);//相机畸变参数
    cv::Mat iner = (cv::Mat_<double>(3, 3) << 313.7605, 0, 156.9846, 0, 313.2382, 124.7260, 0, 0, 1.0000);//相机内参
    //     cv::Mat con = (cv::Mat_<double>(4,1)<<-0.4267,0.2130,0.0,0.0);
     //   cv::Mat iner = (cv::Mat_<double>(3,3)<<660.3323,0,304.6620,0,655.4458,243.3709,0,0,1);
	cv::Mat map1, map2;
    cv::initUndistortRectifyMap(iner,con,cv::Mat(),iner,cv::Size(camFormat.width,camFormat.height),CV_16SC2,map1,map2);
    while (ros::ok()) {
        int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        if (size == -1) {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }
        cv::remap(srcImg,srcImg,map1,map2,cv::INTER_LINEAR);
		cv::cvtColor(srcImg,gray,CV_YUV2GRAY_YUYV);
        //cv::cvtColor(srcImg,gray,CV_YUV2BGR_YUYV);//转为BGR图像
        ros::Time now = ros::Time::now();

        header.stamp=now;
        img_msg=cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
        //img_msg=cv_bridge::CvImage(header, "bgr8",gray ).toImageMsg();//bgr图像传输
        pub_img.publish(img_msg);

 
        ros::spinOnce();
    }

    return 0;
}


