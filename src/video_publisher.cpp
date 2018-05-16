#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    int camera_id;
    if (!ros::param::get("~camera_id", camera_id))
    {
		ROS_ERROR("Could not get the parameter camera_id\n.");
		return 1;
    }
    cv::VideoCapture cap(camera_id);
    if(!cap.isOpened()) 
    {
		ROS_ERROR("Could not open the camera with id %d.", camera_id);
		return 1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

	int rate;
	if (!ros::param::get("~loop_rate", rate))
    {
		ROS_ERROR("Could not get the parameter rate\n.");
		return 1;
    }
    ros::Rate loop_rate(rate);
    while (nh.ok()) {
		cap >> frame;
		if(!frame.empty()) {
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			pub.publish(msg);
		}
		loop_rate.sleep();
    }
    cap.release();
    return 0;
}