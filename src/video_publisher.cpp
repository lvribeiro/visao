#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera", 1);

    int camera_id;
    cv::VideoCapture cap;
    if (_nh.getParam("camera_id", camera_id))
    {
        ROS_INFO_STREAM("Getting video from provider: /dev/video" << camera_id);
        cap.open(camera_id);
    }else{
        ROS_ERROR("Failed to get param 'camera_id'");
        return -1;
    }


    int fps;
    _nh.param("fps", fps, 240);
    ROS_INFO_STREAM("Throttling to fps: " << fps);


    int width_target;
    int height_target;
    _nh.param("width", width_target, 0);
    _nh.param("height", height_target, 0);
    if (width_target != 0 && height_target != 0){
        ROS_INFO_STREAM("Forced image width is: " << width_target);
        ROS_INFO_STREAM("Forced image height is: " << height_target);
    }

    if(!cap.isOpened()){
        ROS_ERROR_STREAM("Could not open the stream.");
        return -1;
    }
    if (width_target != 0 && height_target != 0){
        cap.set(CV_CAP_PROP_FRAME_WIDTH, width_target);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, height_target);
    }

    int cvt2gray;
    _nh.param("cvt2gray", cvt2gray, 0);


    ROS_INFO_STREAM("Opened the stream, starting to publish.");

    cv::Mat frame, gray;
    sensor_msgs::ImagePtr msg;
    std_msgs::Header header;
	
    ros::Rate loop_rate(fps);
    while (nh.ok()) {
		cap >> frame;
        if (pub.getNumSubscribers() > 0)
        {
		    if(!frame.empty()) {
                if (cvt2gray != 0)
                {
		    	    cv::cvtColor(frame, gray, CV_BGR2GRAY);
		    	    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
                }
                else
                {
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                }
                pub.publish(msg);
		    }
            ros::spinOnce();
        }
        loop_rate.sleep();
    }
    cap.release();
    return 0;
}