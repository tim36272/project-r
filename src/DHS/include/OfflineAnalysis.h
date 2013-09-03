#ifndef OFFLINEANALYSIS_H
#define OFFLINEANALYSIS_H

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


class OfflineAnalysis {
public:
	OfflineAnalysis();
	OfflineAnalysis(const std::string& people_topic, const std::string& bags_topic);

	utility::ColorPairListType people_colors_;
	utility::ColorSingleListType bag_colors_;

private:
	//callbacks
	void PersonColorCallback(const dhs::person& msg);
	void BagColorCallback(const dhs::bag& msg);

	ros::NodeHandle node_handle_;

	ros::Subscriber person_subscription_,bag_subscription_;
};


#endif
