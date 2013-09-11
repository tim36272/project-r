/*
 * Broadcaster.h
 *
 *  Created on: Sep 10, 2013
 *      Author: tim
 */

#ifndef BROADCASTER_H_
#define BROADCASTER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class Broadcaster {
public:
	Broadcaster() : it_(nh_) {
		image_pub_ = it_.advertise("/tim/result", 1);
	}
	void Broadcast(const cv::Mat& frame) {
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
		cv_ptr->image = frame;
		cv_ptr->encoding = "bgr8";
		image_pub_.publish(cv_ptr->toImageMsg());
	}
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;
};



#endif /* BROADCASTER_H_ */
