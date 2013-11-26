/*
 * ImageSender.cpp
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
#include "ImageSender.h"
/*
 * Sets up the ros publisher, the queue size is set to a small number because
 * the transport stream shouldn't get too far behind real time
 */
ImageSender::ImageSender(const std::string& image_topic_name):transport_(handle_) {
	image_pub_ = transport_.advertise(image_topic_name,1);
//	if(!image_pub_) {
		//there was an error, which has already been logged in the ros logger
		//not really much for this constructor to do about it
//	}
}

/*
 * publishes the frame over ROS, even if there are no subscribers
 * calling function should check if there are subscribers before doing
 * excessive processing
 */
void ImageSender::SendFrame(int sequence_number,const cv::Mat& frame) {
	sequence_number_ = sequence_number;
	image_pub_.publish(ConvertMatToMsg(frame));
}
/*
 * Returns true if there is a node subscribed to this publisher
 * Technically, returns the number of publishers cast as a boolean
 * Where 0==false, anything else==true
 * To avoid comparison
 */
bool ImageSender::HasSubscriber() {
	return image_pub_.getNumSubscribers();
}

sensor_msgs::ImageConstPtr ImageSender::ConvertMatToMsg(const cv::Mat& frame){
	std_msgs::Header header;
	header.seq = sequence_number_;
	std::string encoding;

	assert(frame.channels()==1 || frame.channels()==3);
	if(frame.channels()==1) encoding="mono8";
	else if(frame.channels()==3) encoding="bgr8";

	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(header,encoding,frame));
	return cv_ptr->toImageMsg();
}
