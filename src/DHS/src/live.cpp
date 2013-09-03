#include <ros/ros.h>
#include "MessageFetcher.h"
#include "Analysis.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utilities.cpp"
#include "Types.h"
#include "InstanceGrabber.h"

static const std::string kWindow = "Live";
//static const std::string kDepthWindow = "depth";
static const int kFontThickness = 2;
static const double kFontScale = 0.6;

static const bool kRecord = true;



void drawPeople(const PersonList& people,cv::Mat* image, int frame_number);
void drawBags(const PersonList& people,const BagList& bags, cv::Mat* image, int frame_number);
int getVideoNumber(const char path[]);
void DrawInstancerLabels(int objective_code,const InstanceGrabber& instancer,cv::Mat* image);
void SetupEventSelecter(int* event_type);
void onMouseEventSelector(int event,int x, int y, int flags, void* data);

int main(int argc, char** argv)
{
	cv::namedWindow(kWindow,CV_WINDOW_AUTOSIZE);
	cv::moveWindow(kWindow,1000,20);

	ros::init(argc, argv, "live");

	MessageFetcher ros_handle;
	cv::Mat color_raw,depth_raw,hsv_raw;
	Analysis analysis_handle("/r/people","/r/bags");
	PersonList people;
	BagList bags;
	bool run = true;
	int frame_number;
	InstanceGrabber instancer;
	bool instancer_setup;
	
	//create output writer
	cv::VideoWriter writer;
	std::stringstream video_output_name;
	video_output_name << "saves/save_"<<getVideoNumber("saves/video_number")<<".avi";
	if(kRecord) writer.open(video_output_name.str(),CV_FOURCC('D','I','V','X'),30,cv::Size(640,480),true);

	int event_type = -1;
	//make a selecter
	SetupEventSelecter(&event_type);

	//main loop
	while(ros::ok() && run) {
		//spin until we get a new frame
		if(!ros_handle.GetFrame(color_raw,depth_raw)) {
			ros::spinOnce();
			if(cv::waitKey(1)=='q') {
				run=false;
			}
			continue;
		}
		//blur the input
		cv::GaussianBlur(color_raw,color_raw,cv::Size(5,5),0,0);
		cvtColor(color_raw,hsv_raw,CV_BGR2HSV);

		//blur the input for analysis
		cv::Mat color_raw_blurred;
		cv::GaussianBlur(color_raw,color_raw_blurred,cv::Size(11,11),0,0);
		//look for the colors
		frame_number = analysis_handle.Update(color_raw_blurred,depth_raw,&people,&bags);

		if(instancer_setup) {
			//update instancer
			instancer.Update(frame_number,people,bags);
		}
		else { //try to setup the instancer
			switch(event_type) {
			case -1:
				//don't do anything
			break;
			case 0:
				if(people.size()>0){ 				instancer.Setup(R_ONE_PERSON_TRACK,			people[0].color[0],people[0].color[1]);
					instancer_setup = true;
				}
			break;
			case 1:
				if(people.size()>0 &&bags.size()>0){instancer.Setup(R_ONE_PERSON_BAG_UNATTENDED,people[0].color[0],people[0].color[1],bags[0].color);
					instancer_setup = true;
				}
			break;
			case 2:
				if(people.size()>1){					instancer.Setup(R_TWO_PERSON_MEET,			people[0].color[0],people[0].color[1],people[1].color[0],people[1].color[1]);
					instancer_setup = true;
				}
			break;
			case 3:
				if(people.size()>1 &&bags.size()>0){instancer.Setup(R_TWO_PERSON_BAG_STEAL,		people[0].color[0],people[0].color[1],people[1].color[0],people[1].color[1],bags[0].color);
					instancer_setup = true;
				}
			break;
			case 4:
				if(people.size()>1 &&bags.size()>0){instancer.Setup(R_TWO_PERSON_BAG_EXCHANGE,	people[0].color[0],people[0].color[1],people[1].color[0],people[1].color[1],bags[0].color);
					instancer_setup = true;
				}
			break;

			}
		}

		//show the blobs on the frame
		cv::Mat output;
		color_raw.copyTo(output);
		cv::rectangle(output,cv::Rect(0,0,color_raw.size().width,100),cv::Scalar(0,0,0),-1,8);
		drawPeople(people,&output,frame_number);
		drawBags(people,bags,&output,frame_number);
		DrawInstancerLabels(event_type,instancer,&output);

		//save the video, if applicable
		//Note:: this has annotations.
		if(kRecord) writer.write(output);

		//show the image on screen
		imshow(kWindow,output);

		char key = cv::waitKey(1);
		switch(key) {
		case 'q':
		case 'Q':
			run = false;
			break;
		case 'd':
			//dump the lists
			people.clear();
			bags.clear();
			analysis_handle.dump();
			break;
		}
	}
	//cleanup
	cv::destroyAllWindows();

	//save instances
	instancer.PrintInstances(std::cout);

	return 0;
}

void drawPeople(const PersonList& people,cv::Mat* image, int frame_number) {
	for(unsigned int person_index=0;person_index<people.size();person_index++) {
		//if this was seen in the last 3 frames
		cv::Scalar rect_color;
		if((frame_number-people[person_index].first_seen) < 5) rect_color = cv::Scalar(0,255,255);
		else if(frame_number-people[person_index].last_seen > 10) continue;
		else if(frame_number-people[person_index].last_seen > 2) rect_color = cv::Scalar(0,0,255);
		else rect_color = cv::Scalar(0,255,0);

			std::stringstream text;
			text << "Person #"<<person_index;

			//draw Kalman estimates
			cv::Point estimate_origin = people[person_index].filter.bounding_rect_origin();

			cv::putText(*image,text.str(),estimate_origin,CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
			cv::rectangle(*image,people[person_index].filter.bounding_rect(),rect_color,1,8);
	}
	int x_origin=5;
	int y_origin=15;
	std::stringstream number_found;
	number_found<<"People found: "<<people.size();
	cv::putText(*image,number_found.str(),cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);

}

void drawBags(const PersonList& people,const BagList& bags, cv::Mat* image, int frame_number) {
	for(unsigned int bag_index=0;bag_index<bags.size();bag_index++) {
		//if this was seen in the last 3 frames
		cv::Scalar rect_color;
		if(frame_number-bags[bag_index].first_seen < 5) rect_color = cv::Scalar(255,255,0);
		else if((frame_number-bags[bag_index].last_seen) > 10) continue;
		else if((frame_number-bags[bag_index].last_seen) > 6) rect_color = cv::Scalar(0,0,255);
		else rect_color = cv::Scalar(0,255,0);
			std::stringstream text;
			text << "Bag #"<<bag_index;

			//draw Kalman estimates
			cv::Point estimate_origin = bags[bag_index].filter.bounding_rect_origin();

			cv::putText(*image,text.str(),estimate_origin,CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
			cv::rectangle(*image,bags[bag_index].filter.bounding_rect(),rect_color,1,8);

			//draw line between the bag and owner
			cv::Point owner_center,bag_center;
			int owner_index = bags[bag_index].belongs_to;
			if(owner_index!=-1 && (int(people.size())-1)>=owner_index) {
				if((frame_number-people[owner_index].last_seen) < 10 ) {
					owner_center.x = people[owner_index].filter.bounding_rect().x+people[owner_index].filter.bounding_rect().width/2;
					owner_center.y = people[owner_index].filter.bounding_rect().y+people[owner_index].filter.bounding_rect().height/2;

					bag_center.x = bags[bag_index].filter.bounding_rect().x+bags[bag_index].filter.bounding_rect().width/2;
					bag_center.y = bags[bag_index].filter.bounding_rect().y+bags[bag_index].filter.bounding_rect().height/2;
					cv::line(*image,bag_center,owner_center,cv::Scalar(0,255,0),2,8);
				}
			}
	}
	int x_origin=5;
	int y_origin=35;
	std::stringstream number_found;
	number_found<<"Bags found: "<<bags.size();
	cv::putText(*image,number_found.str(),cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
}

int getVideoNumber(const char path[]) {
	std::ifstream fin;
	fin.clear();
	fin.open(path);
	int number;
	if(fin.good()) {
		fin >> number;
		fin.close();
	}
	else {
		number = 0;
	}
	std::ofstream fout;
	fout.clear();
	fout.open(path);
	fout << (number+1);
	fout.close();
	return number;
}

void DrawInstancerLabels(int objective_code,const InstanceGrabber& instancer,cv::Mat* image) {
	if(objective_code==R_NO_EVENT) return;
	//check if the event is occuring
	//draw initial label
	int x_origin=5;
	int y_origin=55;

	std::string active_text,label;
		switch(objective_code) {
		case R_ONE_PERSON_TRACK:
			label = "Searching for: one person track";
			active_text = "Person 0 being tracked";
			break;
		case R_ONE_PERSON_BAG_UNATTENDED:
			label = "Searching for: unattended bag";
			active_text = "Person 0 left bag 0 unattended";
			break;
		case R_TWO_PERSON_MEET:
			label = "Searching for: two people meeting";
			active_text = "Person 0 is meeting person 1";
			break;
		case R_TWO_PERSON_BAG_STEAL:
			label = "Searching for: bag stolen";
			active_text = "Person 1 might be stealing person 0's bag";
			break;
		case R_TWO_PERSON_BAG_EXCHANGE:
			label = "Searching for: bag_exchange";
			active_text = "Person 1 is receiving a bag from person 0";
			break;
		default:
			//Incorrect objective code
			assert(false);
			break;
		}
	cv::putText(*image,label,cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255,0,0),kFontThickness,8);
	y_origin +=20;

	if( instancer.EventInProgress() ) {
		cv::putText(*image,active_text,cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
	}
	else {
		if(objective_code==R_TWO_PERSON_BAG_STEAL && instancer.AlreadyStolen()) {
			cv::putText(*image,"Person 1 stole a bag from person 0",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
		}
		else {
			cv::putText(*image,"No events to report",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,255,0),kFontThickness,8);

		}
	}
}

void SetupEventSelecter(int* event_type) {
	cv::Mat selector_window(cv::Size(300,100),CV_8UC1,cv::Scalar(0));
	int x_origin=20,y_origin=15;
	cv::rectangle(selector_window,cv::Rect(5,y_origin+2-12,8,8),cv::Scalar(255),-1,8);
	cv::rectangle(selector_window,cv::Rect(7,y_origin+4-12,4,4),cv::Scalar(128),-1,8);
	cv::putText(selector_window,"One person search",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
	y_origin+=20;

	cv::rectangle(selector_window,cv::Rect(5,y_origin+2-12,8,8),cv::Scalar(255),-1,8);
	cv::rectangle(selector_window,cv::Rect(7,y_origin+4-12,4,4),cv::Scalar(128),-1,8);
	cv::putText(selector_window,"One person bag unattended",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
	y_origin+=20;

	cv::rectangle(selector_window,cv::Rect(5,y_origin+2-12,8,8),cv::Scalar(255),-1,8);
	cv::rectangle(selector_window,cv::Rect(7,y_origin+4-12,4,4),cv::Scalar(128),-1,8);
	cv::putText(selector_window,"Two person meet",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
	y_origin+=20;

	cv::rectangle(selector_window,cv::Rect(5,y_origin+2-12,8,8),cv::Scalar(255),-1,8);
	cv::rectangle(selector_window,cv::Rect(7,y_origin+4-12,4,4),cv::Scalar(128),-1,8);
	cv::putText(selector_window,"Two person bag steal",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
	y_origin+=20;

	cv::rectangle(selector_window,cv::Rect(5,y_origin+2-12,8,8),cv::Scalar(255),-1,8);
	cv::rectangle(selector_window,cv::Rect(7,y_origin+4-12,4,4),cv::Scalar(128),-1,8);
	cv::putText(selector_window,"Two person bag exchange",cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
	imshow("Event Selector",selector_window);
	cv::setMouseCallback("Event Selector",onMouseEventSelector,event_type);
}

void onMouseEventSelector(int event,int x, int y, int flags, void* data) {
	if(event==CV_EVENT_LBUTTONDOWN){
		int* input = (int*)data;
		*input = y/20;
	}
}
