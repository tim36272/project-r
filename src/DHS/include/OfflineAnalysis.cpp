#include "OfflineAnalysis.h"

OfflineAnalysis::OfflineAnalysis(const std::string& people_topic, const std::string& bags_topic){

	//subscribe to color topics
	person_subscription_ = node_handle_.subscribe(people_topic, 10, &OfflineAnalysis::PersonColorCallback,this);
	bag_subscription_ = node_handle_.subscribe(bags_topic, 10, &OfflineAnalysis::BagColorCallback,this);
	std::cout<<"setup callbacks"<<std::endl;
}
void OfflineAnalysis::PersonColorCallback(const dhs::person& msg) {
	std::cout<<"Caught person message"<<std::endl;
	utility::ColorPair temp;

	temp[0][0] = msg.upper_blue;
	temp[0][1] = msg.upper_green;
	temp[0][2] = msg.upper_red;

	temp[1].val[0] = msg.lower_blue;
	temp[1].val[1] = msg.lower_green;
	temp[1].val[2] = msg.lower_red;
	people_colors_.push_back(temp);
}
void OfflineAnalysis::BagColorCallback(const dhs::bag& msg) {
	std::cout<<"Caught bag message"<<std::endl;
	cv::Scalar temp;
	temp[0] = msg.blue;
	temp[1] = msg.green;
	temp[2] = msg.red;
	bag_colors_.push_back(temp);
}
