#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <vector>
#include "utility.h"
#include "event.cpp"

int main() {
	std::vector<cv::VideoCapture> videos;

	//load all the video names
	std::ifstream fin;
	fin.open("offline/data.txt");
	std::string video_name;
	fin >> video_name;
	while(fin.good()) {
		videos.push_back(cv::VideoCapture(video_name));
		fin >> video_name;
	}
	cv::Mat menu(cv::Size(350,20*videos.size()+20),CV_8UC1,cv::Scalar(0));
	cv::Point top_left(5,0);
	MakeButton(&menu,"Select and instance to view",top_left,cv::Scalar(0));
	top_left.y +=20;
	for(int i=0;i<videos.size();i++) {
		std::stringstream name;
		name<<"Instance "<<i+1;
		MakeButton(&menu,name.str(),top_left,cv::Scalar(0));
		top_left.y +=20;
	}
	imshow("Menu",menu);
	int event=-1,last_event = -1;
	cv::setMouseCallback("Menu",onMouseEventSelector,&event);
	bool run = true;
	while(event==-1) cv::waitKey(1);
	while(run) {
		if(event!=last_event && last_event!=-1) {
			videos[last_event].set(CV_CAP_PROP_POS_FRAMES,0);
			last_event = event;
		}
		//show the active video
		cv::Mat input;
		videos[event] >> input;
		if(input.data) imshow("Video",input);
		if(cv::waitKey(33)=='q') run = false;
	}

	return 0;

}
