/*
 * ImageFetcher.cpp
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
#include "ImageFetcher.h"
ImageFetcher::ImageFetcher(const std::string& topic_name) : transport_(handle_)
{
	//transport setup
	subscription_ = transport_.subscribe(topic_name, 1, &ImageFetcher::FrameCallback,this);
	sleep(1);
	ROS_INFO_STREAM("Num publishers: "<<subscription_.getNumPublishers());
	updated_= false;

	sequence_number_ = 0;
}

ImageFetcher::ImageFetcher() : transport_(handle_)
{
	updated_= false;

	sequence_number_ = 0;
}

void ImageFetcher::FrameCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//convert the message to CvImagePtr
	convertMsgToCvImagePtr(msg,raw_frame_ptr_);

	//If the frame is not an 8 bit unsigned Mat (typically 16 bit float Mat
	//from the depth camera) convert and scale it
	if(raw_frame_ptr_->image.depth()!=CV_8U) {
		//8000 is used because that is typically the max value from the Kinect depth camera
		//TODO:find a better way to pick a number for this
		raw_frame_ptr_->image.convertTo(raw_frame_ptr_->image,CV_8U,255./8000.);
	}

	updated_ = true;
}

int ImageFetcher::GetFrame(cv::Mat& frame) {
	if(!updated_) {
		return FRAME_NOT_UPDATED;
	}

	//update sequence number
	if((raw_frame_ptr_->header.seq - sequence_number_)>1) {
		if(sequence_number_!=0) {
			ROS_WARN_STREAM(raw_frame_ptr_->header.seq - sequence_number_-1 << " depth frame(s) were dropped");
		}
	}
	sequence_number_ = raw_frame_ptr_->header.seq;

	frame = raw_frame_ptr_->image;
	updated_=false;
	return sequence_number_;
}

void ImageFetcher::convertMsgToCvImagePtr(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& raw_ptr){
	//try to get the message
	try
	{
		raw_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

std::string ImageFetcher::getTopicName() const{
	return subscription_.getTopic();
}
