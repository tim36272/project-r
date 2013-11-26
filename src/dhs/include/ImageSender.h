/*
 * ImageSender.h
    Copyright (C) 2013  Timothy Sweet

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef IMAGESENDER_H_
#define IMAGESENDER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "common.h"
class ImageSender
{
	private:
		//transport handles
		ros::NodeHandle handle_;
		image_transport::ImageTransport transport_;
		image_transport::Publisher image_pub_;
		int sequence_number_;


		sensor_msgs::ImageConstPtr ConvertMatToMsg(const cv::Mat& frame);

	public:
		DISALLOW_COPY_AND_ASSIGN(ImageSender);
		void SendFrame(int sequence_number,const cv::Mat& frame);
		//sets up a ros publisher on image_topic_name
		ImageSender(const std::string& image_topic_name);
		bool HasSubscriber();
};



#endif /* MESSAGESENDER_H_ */
