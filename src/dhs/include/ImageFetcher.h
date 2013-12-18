/*
 * ImageFetcher.h
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
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "common.h"

#define FRAME_NOT_UPDATED -1
/*
 * grabs a frame on a specified topic and provides it as a cv::Mat
 */
class ImageFetcher
{
	public:
		//allow default construction which doesn't subscribe to anything
		ImageFetcher();
		//the usual constructor
		ImageFetcher(const std::string& topic_name);
		DISALLOW_COPY_AND_ASSIGN(ImageFetcher);
		/*
		 * Returns sequence number, or 0 if there is no frame
		 * Potentially discards the first frame
		 */
		int GetFrame(cv::Mat& frame);
		/*
		 * Returns the most recent frame in memory, regardless of how old it is
		 * or 0 if there is no frame
		 */
		int GetMostRecentFrame(cv::Mat& frame);
		inline bool IsUpdated() {return updated_;}
		std::string getTopicName() const;

	private:
		//transport handles
		ros::NodeHandle handle_;
		image_transport::ImageTransport transport_;
		image_transport::Subscriber subscription_;

		//pointers to raw Mats
		cv_bridge::CvImagePtr raw_frame_ptr_;

		void FrameCallback(const sensor_msgs::ImageConstPtr& msg);
		bool updated_;
		void convertMsgToCvImagePtr(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& raw_ptr);

		int sequence_number_;

		void printSubscriberCount(const ros::WallTimerEvent&);
		ros::WallTimer startup_timer_;
};
