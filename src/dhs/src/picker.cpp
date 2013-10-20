#include <ros/ros.h>
#include "MessageFetcher.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utility.h"
#include "dhs/person.h"
#include "dhs/bag.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define SHOW_DEPTH true

//#include "Types.h"

static const std::string kWindow = "Picker";
static const std::string kProjections = "Projections";
static const std::string kDepth = "Depth";


void SetupPicker(utility::ColorListType* person_list,utility::ColorListType* bag_list);
void OnMouse(int event,int x,int y,int flags,void* data);

struct color_triple {
	cv::Scalar upper;
	cv::Scalar lower;
	cv::Scalar bag;
	cv::Mat image;
	bool upper_dirty;
	bool lower_dirty;
	bool bag_dirty;
	color_triple() {
		upper_dirty = false;
		lower_dirty = false;
		bag_dirty = false;
	}
};


int main(int argc, char** argv)
{
	cv::namedWindow(kWindow,CV_WINDOW_AUTOSIZE);
	cv::namedWindow(kProjections,CV_WINDOW_AUTOSIZE);
if(SHOW_DEPTH)	cv::namedWindow(kDepth,CV_WINDOW_AUTOSIZE);
	cv::moveWindow(kWindow,1400,20);
	cv::moveWindow(kProjections,2050,20);
	if(SHOW_DEPTH)	cv::moveWindow(kDepth,2700,20);

	ros::init(argc, argv, "picker");

	MessageFetcher ros_handle;
	cv::Mat color_raw(cv::Size(680,420),CV_8UC3,cv::Scalar(0,0,0)),depth_raw,hsv_raw;
	color_triple data;
	data.image = color_raw;
	cv::setMouseCallback(kWindow,OnMouse,&data);
	
	bool run = true;

	//setup pickers
	utility::ColorListType bag_list(1);
	utility::ColorListType person_list(2);
		//make a vector of however many people color pairs we are interested in
			//preload people colors
		/*
			person_list[0].color = cv::Scalar(0,244,178);
			person_list[1].color = cv::Scalar(236,11,0);
			person_list[0].dirty = true;
			person_list[1].dirty = true;

			person_list[2].color = cv::Scalar(68,0,240);
			person_list[3].color = cv::Scalar(0,159,241);
			person_list[2].dirty = true;
			person_list[3].dirty = true;*/
		//and the same for bags
		//setup
		SetupPicker(&person_list,&bag_list);

		//setup publisher
		ros::NodeHandle publisher_node;

		ros::Publisher person_publisher = publisher_node.advertise<dhs::person>("r/people",1000);
		ros::Publisher bag_publisher = publisher_node.advertise<dhs::bag>("r/bags",1000);
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
//			cv::rectangle(color_raw,cv::Point(600,282),cv::Point(680,400),cv::Scalar(0,0,0),-1,8);
			data.image = color_raw;
			if(SHOW_DEPTH) {
	//			depth_raw.convertTo(depth_raw,CV_8U,255./2000.);
				imshow(kDepth,depth_raw);
			}
		}
		//blur the input

		cv::Mat output;
		color_raw.copyTo(output);

		//make HSV version
		cvtColor(color_raw,hsv_raw,CV_BGR2HSV);
/*		imshow("HSV",hsv_raw);
		std::vector<cv::Mat> planes;
		cv::split(hsv_raw,planes);
		imshow("Hue",planes[0]);
		imshow("Sat",planes[1]);
		imshow("Val",planes[2]);*/




		//update from picker
		if(data.upper_dirty) {
			data.upper_dirty = false;
			person_list[0].color = data.upper;
			person_list[0].dirty = true;
		}
		if(data.lower_dirty) {
			data.lower_dirty = false;
			person_list[1].color = data.lower;
			person_list[1].dirty = true;
		}
		if(data.bag_dirty) {
			data.bag_dirty = false;
			bag_list[0].color = data.bag;
			bag_list[0].dirty = true;
		}

		//show colors
		cv::Mat person(cv::Size(200,200),CV_8UC3,person_list[0].color);
		cv::Rect lower(0,100,200,100);
		cv::rectangle(person,lower,person_list[1].color,-1,8);
		cv::cvtColor(person,person,CV_HSV2BGR);
		imshow("person",person);
		cv::Mat bag(cv::Size(100,100),CV_8UC3,bag_list[0].color);
		cv::cvtColor(bag,bag,CV_HSV2BGR);
		imshow("bag",bag);

		//show current projections
		cv::Mat projections;
//		utility::AddCurrentProjection(person_list[0].color,hsv_raw,&projections); //uncomment this and comment loops below and dirty checks above for only upper


		for(uint person_index=0;person_index<person_list.size();person_index++) {
			utility::AddCurrentProjection(person_list[person_index].color,hsv_raw,&projections);
		}

		for(uint bag_index=0;bag_index<bag_list.size();bag_index+=2) {
			utility::AddCurrentProjection(bag_list[bag_index].color,hsv_raw,&projections);
		}
		imshow(kProjections,projections);

		//publish dirty messages
		for(uint person_index=0;person_index<person_list.size();person_index+=2) {
			if(person_list[person_index].dirty && person_list[person_index+1].dirty) {
				std::cout<<"sending people list"<<std::endl;
				person_list[person_index].dirty = false;
				person_list[person_index+1].dirty = false;


				dhs::person msg;
				msg.upper_blue = person_list[person_index].color[0];
				msg.upper_green = person_list[person_index].color[1];
				msg.upper_red = person_list[person_index].color[2];
				msg.lower_blue = person_list[person_index+1].color[0];
				msg.lower_green = person_list[person_index+1].color[1];
				msg.lower_red = person_list[person_index+1].color[2];

				/*
				dhs::person msg;
				msg.upper_blue = 177;
				msg.upper_green = 252;
				msg.upper_red = 84;
				msg.lower_blue = 129;
				msg.lower_green = 54;
				msg.lower_red = 61;
*/
				person_publisher.publish(msg);
				ros::spinOnce();
			}
		}
		for(uint bag_index=0;bag_index<bag_list.size();bag_index++) {
			if(bag_list[bag_index].dirty) {
				std::cout<<"sending bag list"<<std::endl;
				bag_list[bag_index].dirty = false;
				dhs::bag msg;
				msg.blue = bag_list[bag_index].color[0];
				msg.green = bag_list[bag_index].color[1];
				msg.red = bag_list[bag_index].color[2];

				bag_publisher.publish(msg);
				ros::spinOnce();
			}
		}

		//show the image on screen
		imshow(kWindow,output);
//		imshow(kDepthWindow,depth_raw);


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

void SetupPicker(utility::ColorListType* person_list,utility::ColorListType* bag_list) {
	assert(person_list->size()%2==0);
	//setup person pickers
	cv::namedWindow("person",CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("Upper H","person",&((*person_list)[0].color[0]),180,NULL);
	cv::createTrackbar("Upper S","person",&((*person_list)[0].color[1]),255,NULL);
	cv::createTrackbar("Upper V","person",&((*person_list)[0].color[2]),255,NULL);
	cv::createTrackbar("Lower H","person",&((*person_list)[1].color[0]),180,NULL);
	cv::createTrackbar("Lower S","person",&((*person_list)[1].color[1]),255,NULL);
	cv::createTrackbar("Lower V","person",&((*person_list)[1].color[2]),255,NULL);
	cv::createTrackbar("Send","person",&((*person_list)[0].dirty),1,NULL);

	cv::namedWindow("bag",CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("Bag H","bag",&((*bag_list)[0].color[0]),180,NULL);
	cv::createTrackbar("Bag S","bag",&((*bag_list)[0].color[1]),255,NULL);
	cv::createTrackbar("Bag V","bag",&((*bag_list)[0].color[2]),255,NULL);
	cv::createTrackbar("Send","bag",&((*bag_list)[0].dirty),1,NULL);

	cv::moveWindow("person",70,20);
	cv::moveWindow("bag",70,350);
}

void OnMouse(int event,int x,int y,int flags,void* input) {
	color_triple* data = (color_triple*)input;
	cv::Scalar rgb_sample;

	int x_window=20,y_window=20;
	rgb_sample[0] = data->image.at<cv::Vec3b>(y,x)[0];
	rgb_sample[1] = data->image.at<cv::Vec3b>(y,x)[1];
	rgb_sample[2] = data->image.at<cv::Vec3b>(y,x)[2];

	for(int c=0;c<3;c++) {
		for(int i=0;i<x_window;i++) {
			for(int j=0;j<y_window;j++) {
				rgb_sample[c] += data->image.at<cv::Vec3b>(y-y_window/2+i,x-x_window/2+j)[c];
			}
		}
	}
	rgb_sample[0] /=x_window*y_window+1;
	rgb_sample[1] /=x_window*y_window+1;
	rgb_sample[2] /=x_window*y_window+1;

	if(event==CV_EVENT_LBUTTONDOWN && (flags/16)%2!=1 ) {
		data->upper_dirty = true;
		data->upper = utility::BGR2HSV(rgb_sample);
	}
	else if(event==CV_EVENT_LBUTTONUP && flags==CV_EVENT_FLAG_LBUTTON) {
		data->lower_dirty = true;
		data->lower = utility::BGR2HSV(rgb_sample);
	}
	else if(event==CV_EVENT_LBUTTONDOWN) {
		data->bag_dirty = true;
		data->bag = utility::BGR2HSV(rgb_sample);
	}
}
