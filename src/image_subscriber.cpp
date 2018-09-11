#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher image_freq;
cv::Mat image, filtered, edges;
std_msgs::Empty empty_msg;

std::vector<cv::Vec2f> lines;

ros::Time b_ini, c_ini, h_ini, ini;
ros::Duration b_dur, c_dur, h_dur, total_dur;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        ini = ros::Time::now();
        image = cv_bridge::toCvShare(msg, "bgr8")->image;

        b_ini = ros::Time::now();
        cv::blur(image, filtered, cv::Size(3, 3));
        b_dur = ros::Time::now() - b_ini;

        c_ini = ros::Time::now();
        cv::Canny(filtered, edges, 100, 200);
        c_dur = ros::Time::now() - c_ini;

        h_ini = ros::Time::now();
        cv::HoughLines(edges, lines, 1, CV_PI / 180.0, 100);
        h_dur = ros::Time::now() - h_ini;
        total_dur = ros::Time::now() - ini;

        image_freq.publish(empty_msg);

        //    cv::imshow("view", edges);
        //    cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }
    ROS_INFO("total %.5lf\nblur  %.5lf\ncanny %.5lf\nhough %.5lf\n",
             total_dur.toSec() * 100, b_dur.toSec() * 100, c_dur.toSec() * 100,
             h_dur.toSec() * 100);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    image_freq = nh.advertise<std_msgs::Empty>("image_freq", 10);
    cv::namedWindow("view");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub =
        it.subscribe("raspicam_node/image", 1, imageCallback);
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}