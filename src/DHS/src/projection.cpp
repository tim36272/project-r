
#include <ros/ros.h>
#include "MessageFetcher.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utilities.cpp"
#include "dhs/person.h"
#include "dhs/bag.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include "Types.h"
static const std::string kProjections = "Projections";

class DescriptorGrabber {
public:
	ros::NodeHandle node_handle_;
	ros::Subscriber person_subscription_,bag_subscription_;
	utility::ColorPairListType colors_;
	utility::ColorSingleListType bag_colors_;
	void PersonColorCallback(const dhs::person& msg)  {
		utility::ColorPair temp;
		temp[0][0] = msg.upper_blue;
		temp[0][1] = msg.upper_green;
		temp[0][2] = msg.upper_red;

		temp[1][0] = msg.lower_blue;
		temp[1][1] = msg.lower_green;
		temp[1][2] = msg.lower_red;
		colors_.push_back(temp);
		std::cout<<"Added a person"<<std::endl;
	}
	void BagColorCallback(const dhs::bag& msg) {
		cv::Scalar temp;
		temp[0] = msg.blue;
		temp[0] = msg.green;
		temp[0] = msg.red;
		bag_colors_.push_back(temp);
		std::cout<<"Added a bag"<<std::endl;
	}
	DescriptorGrabber() {
		person_subscription_ = node_handle_.subscribe("/r/people", 10, &DescriptorGrabber::PersonColorCallback,this);
		bag_subscription_ = node_handle_.subscribe("/r/bags", 10, &DescriptorGrabber::BagColorCallback,this);
	}
};


int main(int argc, char** argv)
{
	cv::namedWindow(kProjections,CV_WINDOW_AUTOSIZE);
	ros::init(argc, argv, "picker");

	MessageFetcher ros_handle;
	cv::Mat color_raw(cv::Size(680,420),CV_8UC3,cv::Scalar(0,0,0)),depth_raw,hsv_raw;

	bool run = true;

	//setup receiver
	DescriptorGrabber grabber;

	//main loop
	while(ros::ok() && run) {
		//spin until we get a new frame
		if(!ros_handle.GetFrame(color_raw,depth_raw)) {
			ros::spinOnce();
			if(cv::waitKey(1)=='q') {
				run=false;
			}
//			continue;
		}
		else {
			cv::GaussianBlur(color_raw,color_raw,cv::Size(11,11),0,0);
		}

		cv::Mat output;
		color_raw.copyTo(output);

		//make HSV version
		cvtColor(color_raw,hsv_raw,CV_BGR2HSV);

		//show current projections
		cv::Mat projections(color_raw.size(),CV_8UC3,cv::Scalar(0));
//		utility::AddCurrentProjection(person_list[0].color,hsv_raw,&projections); //uncomment this and comment loops below and dirty checks above for only upper


		for(uint person_index=0;person_index<grabber.colors_.size();person_index++) {
			utility::AddCurrentProjection(grabber.colors_[person_index][0],hsv_raw,&projections);
			utility::AddCurrentProjection(grabber.colors_[person_index][1],hsv_raw,&projections);

		}

		for(uint bag_index=0;bag_index<grabber.bag_colors_.size();bag_index+=2) {
			utility::AddCurrentProjection(grabber.bag_colors_[bag_index],hsv_raw,&projections);
		}
		imshow(kProjections,projections);

		char key = cv::waitKey(1);
		switch(key) {
		case 'q':
		case 'Q':
			run = false;
			break;
		}
	}
	//cleanup
	cv::destroyAllWindows();

	return 0;
}
