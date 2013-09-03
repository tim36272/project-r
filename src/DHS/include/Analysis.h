#ifndef TIM_SRC_ANALYSIS_H
#define TIM_SRC_ANALYSIS_H

#include <fstream>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <ros/ros.h>

#include "BlobDescriptor.h"
#include "utilities.cpp"
#include "Types.h"
#include "dhs/person.h"
#include "dhs/bag.h"

typedef std::vector<std::vector<cv::Point> > RegionList;

class Analysis {
	public:
		Analysis();
		Analysis(const std::string& people_topic, const std::string& bags_topic);
		int Update(const cv::Mat& color_raw, const cv::Mat& depth_raw, PersonList* known_people, BagList* known_bags);
		utility::ColorPairListType colors_;
		utility::ColorSingleListType bag_colors_;

	private:
		void ScanForNewPeople(const cv::Mat& color_raw, const cv::Mat& hsv_raw, const cv::Mat& depth_raw, PersonList* known_people);
		void ScanForNewBags(const cv::Mat& color_raw, const cv::Mat& hsv_raw, const cv::Mat& depth_raw, const PersonList& known_people, BagList* known_bags);

		//callbacks
		void PersonColorCallback(const dhs::person& msg);
		void BagColorCallback(const dhs::bag& msg);

		int frame_number_;
		cv::Mat color_raw_,depth_raw_;

		ros::NodeHandle node_handle_;

		ros::Subscriber person_subscription_,bag_subscription_;
};

#endif
