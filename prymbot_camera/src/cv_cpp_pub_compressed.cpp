#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_publisher_compressed");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>("/prymbot/image_raw/compressed", 10);

    cv::VideoCapture cap(0, cv::CAP_V4L);
    // Check if video device can be opened with the given index
    if (!cap.isOpened())
    {
        ROS_WARN("No video device found!");
        return 1;
    }

    // set dimensions
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // set FPS
    // cap.set(cv::CAP_PROP_FPS, 60); //CPU intensive. leave default

    cv::Mat frame;

    sensor_msgs::CompressedImage img_msg;

    // set encoder settings for supposed quality
    std::vector<int> encoder_settings = {cv::IMWRITE_JPEG_QUALITY, 25};

    while (nh.ok())
    {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if (!frame.empty())
        {
            img_msg.header.stamp = ros::Time::now();
            img_msg.header.frame_id = "camera_link";
            img_msg.format = "jpeg";
            cv::imencode(".jpg", frame, img_msg.data, encoder_settings);
            pub.publish(img_msg);
        }
        ros::spinOnce();
    }
}