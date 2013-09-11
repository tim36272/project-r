#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <cvaux.h>
class MessageFetcher
{
	private:
		//transport handles
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		ros::Subscriber depth_sub_;
		ros::Subscriber disparity_sub_;
		ros::Subscriber rgb_sub_;

		//pointers to raw Mats
		cv_bridge::CvImagePtr raw_rgb_ptr_;
		cv_bridge::CvImagePtr raw_depth_ptr_;
		cv_bridge::CvImagePtr raw_disparity_ptr_;


		void depthCb(const sensor_msgs::ImageConstPtr& msg);
		void rgbCb(const sensor_msgs::ImageConstPtr& msg);
		void disparityCb(const stereo_msgs::DisparityImageConstPtr& msg);


		bool raw_updated_,depth_updated_;
		void convertMsgToCvImagePtr(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr& raw_ptr);

		int depth_sequence_number_,color_sequence_number_;
	public:
		bool GetFrame(cv::Mat& rgb_frame, cv::Mat& depth_frame);
		MessageFetcher();
		~MessageFetcher();
};
