/*
 * stationary_segmentation.cpp
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

#include "dhs/ImageFetcher.h"
#include "dhs/ImageSender.h"

#define NO_THRESHOLDING -1

class Worker {
public:
	Worker(const std::string& input_topic, const std::string& output_topic) :
		handle_("~"),
		input_stream_(input_topic),
		output_stream_(output_topic) {
		if(!handle_.getParam("/segmentation/use_mog", use_MOG_))
			use_MOG_ = true;
		if(!handle_.getParam("/segmentation/dilate_times", dilate_times_))
				dilate_times_ = 0;
		if(!handle_.getParam("/segmentation/erode_times", erode_times_))
				erode_times_ = 0;
		if(!handle_.getParam("/segmentation/threshold", threshold_))
			threshold_ = NO_THRESHOLDING;
		if(!handle_.getParam("/segmentation/detect_shadows", detect_shadows_))
			detect_shadows_ = false;

		subtractor_.set("nmixtures",3);
		subtractor_.set("backgroundRatio",0.001);
		subtractor_.set("detectShadows",detect_shadows_);
		subtractor_.set("fVarInit",10);
	}

	void callback(const ros::TimerEvent& event);
private:
	ros::NodeHandle handle_;
	ImageFetcher input_stream_;
	ImageSender output_stream_;
	bool use_MOG_,detect_shadows_;
	int threshold_,erode_times_,dilate_times_;
	cv::BackgroundSubtractorMOG2 subtractor_;
	cv::Mat foreground_;
};

int main(int argc, char* argv[]) {
	//connect to ros
	ros::init(argc, argv, "segmentation");
	ros::NodeHandle handle("~");

	Worker worker("in","out");
	//loop at 60 hz since the camera runs half that fast
	ros::Timer timer = handle.createTimer(ros::Duration(1./120.), &Worker::callback, &worker);
	ros::spin();

	return 0;
}

void Worker::callback(const ros::TimerEvent & event) {
	cv::Mat frame;
	//if the frame has been updated
	//aka sequence number is not zero
	int sequence_number = input_stream_.GetFrame(frame);
	if(sequence_number!=FRAME_NOT_UPDATED) {
		//do thresholding if set
		if(threshold_!=NO_THRESHOLDING) {
			//threshold values higher than threshold to 0
			cv::threshold(frame,frame,threshold_,0,cv::THRESH_TOZERO_INV);
		}
		if(use_MOG_) {
			subtractor_.operator ()(frame,foreground_);
			if(detect_shadows_) {
				cv::threshold(foreground_,foreground_,200,255,cv::THRESH_BINARY);
			}
		}
		//dilate and erode the image twice each

		cv::erode(foreground_,foreground_,cv::Mat(),cv::Point(-1,-1),erode_times_);
		cv::dilate(foreground_,foreground_,cv::Mat(),cv::Point(-1,-1),dilate_times_);
		if(output_stream_.HasSubscriber()) {
			output_stream_.SendFrame(sequence_number,foreground_);
		}
	}
}
