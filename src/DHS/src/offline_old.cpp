#include <ros/ros.h>
#include "MessageFetcher.h"
#include "OfflineAnalysis.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utilities.cpp"
#include <vector>
#include "Types.h"

#define NO_FRAMES_RECEIVED -2

#define ONE_PERSON_SEARCH 0
#define TWO_PERSON_SEARCH 1
#define BAG_STOLEN 2
#define BAG_UNATTENDED 3
#define BAG_EXCHANGED 4


//static const std::string kWindow = "output";

void GetVideo(MessageFetcher& ros_handle, std::vector<cv::Mat>* frames);
void VideoPositionCallback(int,void*);
void ButtonCallback(int state,void*);
void ShowMainWindow(int* position,std::vector<cv::Mat>& frames);
void SetupEventPicker(bool* one_person_search, bool* two_person_search, bool* bag_unattended, bool* bag_stolen, bool* bag_exchanged, bool* go);

void ShowPickers(int* first_upper_hue,int* first_lower_hue);
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* bag_hue);
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* second_upper_hue,int* second_lower_hue);
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* second_upper_hue,int* second_lower_hue,int* bag_hue);
void UpdatePicker(int value,void*);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imageProcessor");

	MessageFetcher ros_handle;
	
	bool run = true;
	int frame_number;

	cv::Mat temp(cv::Size(20,20),CV_8UC1,cv::Scalar(0));
	imshow("temp",temp);

	//event picker
	bool one_person_search=false,
		 two_person_search=false,
		 bag_unattended=false,
		 bag_stolen=false,
		 bag_exchanged=false,
		 go=false;
	SetupEventPicker(&one_person_search, &two_person_search, &bag_unattended, &bag_stolen,  &bag_exchanged, &go);

	//get the video from the ROS topic
	std::vector<cv::Mat> frames;
	GetVideo(ros_handle,&frames);

	if(frames.size()==0) {
		std::cout<<"Received no frames"<<std::endl;
		return NO_FRAMES_RECEIVED;
	}

	//show windows
		//main video window
		int video_position=frames.size();
		ShowMainWindow(&video_position,frames);

	//wait for event
	while(go==false) { if(cv::waitKey(1)=='q') break; }

	int event,second_event=0;
	if		(one_person_search)	event = ONE_PERSON_SEARCH;
	else if (two_person_search) event = TWO_PERSON_SEARCH;
	if 		(bag_unattended) 	second_event = BAG_UNATTENDED;
	else if (bag_stolen) 		second_event = BAG_STOLEN;
	else if (bag_exchanged) 	second_event = BAG_EXCHANGED;

	int first_upper_hue=0,first_lower_hue=0,second_upper_hue=0,second_lower_hue=0,bag_hue=0;

	//setup appropriate options
	if(event==ONE_PERSON_SEARCH) {
		if(second_event!=0) {
			ShowPickers(&first_upper_hue,&first_lower_hue);
		}
		else {
			ShowPickers(&first_upper_hue,&first_lower_hue,&bag_hue);
		}
	}
	else {
		if(second_event!=0) {
			ShowPickers(&first_upper_hue,&first_lower_hue,&second_upper_hue,&second_lower_hue);
		}
		else {
			ShowPickers(&first_upper_hue,&first_lower_hue,&second_upper_hue,&second_lower_hue,&bag_hue);
		}
	}



	//main loop
	while(run) {


		char key = cv::waitKey(1);
		switch(key) {
		case 'q': case 'Q':
			run = false;
			break;
		}
	}
	//cleanup
	cv::destroyAllWindows();
	return 0;
}



void GetVideo(MessageFetcher& ros_handle, std::vector<cv::Mat>* frames) {
	cv::Mat color_raw,depth_raw;
	std::cout<<"Please start broadcasting and press any key when complete"<<std::endl;
	while(ros::ok()) {
		//spin until we get a new frame
		if(!ros_handle.GetFrame(color_raw,depth_raw)) {
			ros::spinOnce();
		}
		else {
			frames->push_back(color_raw);

		}
		if(cv::waitKey(1)!=-1) break;
	}
}

void ShowMainWindow(int* position, std::vector<cv::Mat>& frames) {
	cv::Mat temp_black(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
	cv::namedWindow("Video",CV_WINDOW_AUTOSIZE);
	imshow("Video",temp_black);
	int max = *position-1;
	*position = 0;
	cv::createTrackbar("Position","Video",position,max,&VideoPositionCallback,&frames);
	cv::waitKey(1000);
}

void VideoPositionCallback(int position,void* data) {
	std::vector<cv::Mat>* frames = (std::vector<cv::Mat>*)data;
	imshow("Video",(*frames)[position]);
}

void SetupEventPicker(bool* one_person_search, bool* two_person_search, bool* bag_unattended, bool* bag_stolen, bool* bag_exchanged, bool* go) {

	cv::createButton("1-person search",&ButtonCallback,one_person_search,CV_CHECKBOX);
	cv::createButton("2-person search",&ButtonCallback,two_person_search,CV_CHECKBOX);
	cv::createButton("Bag Unattended",&ButtonCallback,bag_unattended,CV_CHECKBOX);
	cv::createButton("Bag stolen",&ButtonCallback,bag_stolen,CV_CHECKBOX);
	cv::createButton("Bag exchanged",&ButtonCallback,bag_exchanged,CV_CHECKBOX);
	cv::createButton("Go",&ButtonCallback,go,CV_PUSH_BUTTON);


}

void ButtonCallback(int state,void* input) {
	bool* data = (bool*)input;
	*data = !(*data);
	std::cout<<"new state: "<<*data<<std::endl;
}

void ShowPickers(int* first_upper_hue,int* first_lower_hue) {
	cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(255,255,255));
	cv::cvtColor(color,color,CV_HSV2BGR);
	imshow("Upper 1",color); imshow("Lower 1",color);

	int* type = new int;
	*type = ONE_PERSON_SEARCH;
	cv::createTrackbar("Upper #1","Upper 1",first_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #1","Lower 1",first_lower_hue,180,&UpdatePicker,type);
}
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* bag_hue) {
	cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(255,255,255));
	cv::cvtColor(color,color,CV_HSV2BGR);
	imshow("Upper 1",color); imshow("Lower 1",color); imshow("Bag",color);

	int* type = new int;
	*type = ONE_PERSON_SEARCH || BAG_STOLEN;
	cv::createTrackbar("Upper #1","Upper 1",first_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #1","Lower 1",first_lower_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Bag","Bag",bag_hue,180,&UpdatePicker);
}
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* second_upper_hue,int* second_lower_hue) {
	cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(255,255,255));
	cv::cvtColor(color,color,CV_HSV2BGR);
	imshow("Upper 1",color); imshow("Lower 1",color); imshow("Upper 2",color); imshow("Lower 2",color);

	int* type = new int;
	*type = TWO_PERSON_SEARCH;
	cv::createTrackbar("Upper #1","Upper 1",first_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #1","Lower 1",first_lower_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Upper #2","Upper 2",second_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #2","Lower 2",second_lower_hue,180,&UpdatePicker,type);
}
void ShowPickers(int* first_upper_hue,int* first_lower_hue,int* second_upper_hue,int* second_lower_hue,int* bag_hue) {
	cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(255,255,255));
	cv::cvtColor(color,color,CV_HSV2BGR);
	imshow("Upper 1",color); imshow("Lower 1",color); imshow("Upper 2",color); imshow("Lower 2",color); imshow("Bag",color);

	int* type = new int;
	*type = TWO_PERSON_SEARCH || BAG_STOLEN;
	cv::createTrackbar("Upper #1","Upper 1",first_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #1","Lower 1",first_lower_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Upper #2","Upper 2",second_upper_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Lower #2","Lower 2",second_lower_hue,180,&UpdatePicker,type);
	cv::createTrackbar("Bag","Bag",bag_hue,180,&UpdatePicker,type);
}

void UpdatePicker(int value,void* data) {
	int* type = (int*)data;

	int one_bag = ONE_PERSON_SEARCH || BAG_STOLEN;
	if(*type==ONE_PERSON_SEARCH) {
		int first_upper_hue = cv::getTrackbarPos("Upper #1","Upper 1");
		int first_lower_hue = cv::getTrackbarPos("Lower #1","Lower 1");

		cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(first_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 1",color);

		color.setTo(cv::Scalar(first_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 1",color);

	}
	else if(*type==(ONE_PERSON_SEARCH || BAG_STOLEN)) {

		int first_upper_hue = cv::getTrackbarPos("Upper #1","Upper 1");
		int first_lower_hue = cv::getTrackbarPos("Lower #1","Lower 1");
		int bag_hue = cv::getTrackbarPos("Bag","Bag");

		cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(first_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 1",color);

		color.setTo(cv::Scalar(first_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 1",color);

		color.setTo(cv::Scalar(bag_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Bag",color);

	}
	else if(*type==TWO_PERSON_SEARCH) {
		int first_upper_hue = cv::getTrackbarPos("Upper #1","Upper 1");
		int first_lower_hue = cv::getTrackbarPos("Lower #1","Lower 1");
		int second_upper_hue = cv::getTrackbarPos("Upper #2","Upper 2");
		int second_lower_hue = cv::getTrackbarPos("Lower #2","Lower 2");

		cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(first_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 1",color);

		color.setTo(cv::Scalar(first_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 1",color);

		color.setTo(cv::Scalar(second_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 2",color);

		color.setTo(cv::Scalar(second_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 2",color);

	}
	else { //two person with bag

		int first_upper_hue = cv::getTrackbarPos("Upper #1","Upper 1");
		int first_lower_hue = cv::getTrackbarPos("Lower #1","Lower 1");
		int second_upper_hue = cv::getTrackbarPos("Upper #2","Upper 2");
		int second_lower_hue = cv::getTrackbarPos("Lower #2","Lower 2");
		int bag_hue = cv::getTrackbarPos("Bag","Bag");

		cv::Mat color(cv::Size(50,50),CV_8UC3,cv::Scalar(first_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 1",color);

		color.setTo(cv::Scalar(first_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 1",color);

		color.setTo(cv::Scalar(second_upper_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Upper 2",color);

		color.setTo(cv::Scalar(second_lower_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Lower 2",color);

		color.setTo(cv::Scalar(bag_hue,255,255));
		cv::cvtColor(color,color,CV_HSV2BGR);
		imshow("Bag",color);

	}
}
