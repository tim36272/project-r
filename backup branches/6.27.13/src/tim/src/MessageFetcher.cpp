#include "MessageFetcher.h"
MessageFetcher::MessageFetcher() : it_(nh_)
{
	//transport setup
	rgb_sub_ = it_.subscribe("camera/rgb/image_color", 1, &MessageFetcher::rgbCb,this);
	depth_sub_ = it_.subscribe("camera/depth_registered/image_raw", 1, &MessageFetcher::depthCb, this);

	raw_updated_= depth_updated_ = false;
}
MessageFetcher::~MessageFetcher() {

}

void MessageFetcher::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	//convert the message to CvImagePtr
	convertMsgToCvImagePtr(msg,raw_depth_ptr_);

	depth_updated_ = true;

}

void MessageFetcher::rgbCb(const sensor_msgs::ImageConstPtr& msg) {
	//convert the message to CvImagePtr
	convertMsgToCvImagePtr(msg,raw_rgb_ptr_);

	raw_updated_ = true;
}

bool MessageFetcher::GetFrame(cv::Mat& rgb_frame, cv::Mat& depth_frame) {
	if(!(raw_updated_ && depth_updated_)) return false;

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
