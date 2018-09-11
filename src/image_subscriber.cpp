#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>

ros::Publisher image_freq;
cv::Mat image, filtered, edges;
std_msgs::Empty empty_msg;

std::vector<cv::Vec2f> lines;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    image =  cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::blur(image, filtered, cv::Size(3,3));
    cv::Canny(filtered, edges, 100, 200);
    cv::HoughLines(edges, lines, 1, CV_PI/180.0, 100);

    image_freq.publish( empty_msg );

//    cv::imshow("view", edges);
//    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  ROS_INFO_STREAM("Hough Transform detected " << lines.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_freq = nh.advertise<std_msgs::Empty>("image_freq", 10);
  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("raspicam_node/image", 1, imageCallback);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}
