#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "TcpClient.h"
#include <iostream>

TcpClient client;

void sender_cb(const sensor_msgs::ImageConstPtr& msg) {
	std::cout<<"in callback"<<std::endl;
	cv_bridge::CvImagePtr raw_ptr;

	try
	{
		raw_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
std::cout<<"Sending image"<<std::endl;
	client.send(raw_ptr->image);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "network_broadcaster");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	ros::Subscriber subscription;

	//get images from topic

    bool b = false;
    while (!b) {
        b = client.connect("25.25.1.200",9001);
        usleep(3e6);
    }

    //start callback
	subscription = nh_.subscribe("/tim/result", 1, &sender_cb);
	while(ros::ok()) {ros::spinOnce();}
}
