#include "MessageFetcher.h"
MessageFetcher::MessageFetcher() : it_(nh_)
{
	//transport setup
	rgb_sub_ = nh_.subscribe("/camera/rgb/image_color", 1, &MessageFetcher::rgbCb,this);
//	disparity_sub_ = nh_.subscribe("/camera/depth_registered/disparity", 1, &MessageFetcher::disparityCb, this);
	depth_sub_ = nh_.subscribe("/camera/depth_registered/image_raw", 1, &MessageFetcher::depthCb, this);

	raw_updated_= depth_updated_ = false;

	depth_sequence_number_ = 0;
	color_sequence_number_ = 0;
}
MessageFetcher::~MessageFetcher() {

}

void MessageFetcher::depthCb(const sensor_msgs::ImageConstPtr& msg)
{

	//convert the message to CvImagePtr
	convertMsgToCvImagePtr(msg,raw_depth_ptr_);

	raw_depth_ptr_->image.convertTo(raw_depth_ptr_->image,CV_8U,255./8000.);

	depth_updated_ = true;

}

void MessageFetcher::rgbCb(const sensor_msgs::ImageConstPtr& msg) {
	//convert the message to CvImagePtr
	convertMsgToCvImagePtr(msg,raw_rgb_ptr_);
	raw_updated_ = true;
}

void MessageFetcher::disparityCb(const stereo_msgs::DisparityImageConstPtr& msg) {
	  try
	  {
		  raw_depth_ptr_ = cv_bridge::toCvCopy(msg->image);
		 // raw_disparity_ptr_ = cv_bridge::toCvCopy(msg->image);

	  }
	  catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	  }
	  depth_updated_ = true;
}

bool MessageFetcher::GetFrame(cv::Mat& rgb_frame, cv::Mat& depth_frame) {
	if(!(raw_updated_ && depth_updated_)) return false;

	//update sequence number
	if((raw_depth_ptr_->header.seq - depth_sequence_number_)>1) {
		if(depth_sequence_number_!=0) {
			ROS_WARN_STREAM(raw_depth_ptr_->header.seq - depth_sequence_number_-1 << " depth frame(s) were dropped");
		}
	}
	if((raw_rgb_ptr_->header.seq - color_sequence_number_)>1) {
		if(color_sequence_number_!=0) {
			ROS_WARN_STREAM(raw_rgb_ptr_->header.seq - color_sequence_number_-1 << " color frame(s) were dropped");
		}
	}
	depth_sequence_number_ = raw_depth_ptr_->header.seq;
	color_sequence_number_ = raw_rgb_ptr_->header.seq;


	rgb_frame = raw_rgb_ptr_->image;
	depth_frame = raw_depth_ptr_->image;
	raw_updated_=depth_updated_=false;
	return true;
}

void MessageFetcher::convertMsgToCvImagePtr(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& raw_ptr){
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
