#include <ros/ros.h>
#include "MessageFetcher.h"
#include "Analysis.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utilities.cpp"
#include "Types.h"

static const std::string kWindow = "Live";
//static const std::string kDepthWindow = "depth";
static const int kFontThickness = 1;
static const double kFontScale = 0.4;

static const bool kRecord = true;



void drawPeople(const PersonList& people,cv::Mat* image, int frame_number);
void drawBags(const PersonList& people,const BagList& bags, cv::Mat* image, int frame_number);
int getVideoNumber(const char path[]);

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
	
	//create output writer
	cv::VideoWriter writer;
	std::stringstream video_output_name;
	video_output_name << "saves/save_"<<getVideoNumber("saves/video_number")<<".avi";
	if(kRecord) writer.open(video_output_name.str(),CV_FOURCC('D','I','V','X'),30,cv::Size(640,480),true);

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
//		cv::rectangle(color_raw,cv::Point(600,282),cv::Point(680,400),cv::Scalar(0,0,0),-1,8);
		cvtColor(color_raw,hsv_raw,CV_BGR2HSV);

		//blur the input for analysis
		cv::Mat color_raw_blurred;
		cv::GaussianBlur(color_raw,color_raw_blurred,cv::Size(11,11),0,0);
		//look for the colors
		frame_number = analysis_handle.Update(color_raw_blurred,depth_raw,&people,&bags);
		std::cout<<"People found: "<<people.size()<<std::endl;
		std::cout<<"Bags found: "<<bags.size()<<std::endl;

		//show the blobs on the frame
		cv::Mat output;
		color_raw.copyTo(output);
		drawPeople(people,&output,frame_number);
		drawBags(people,bags,&output,frame_number);

		//save the video, if applicable
		//Note:: this has annotations.
		if(kRecord) writer.write(output);

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

void drawPeople(const PersonList& people,cv::Mat* image, int frame_number) {
	for(unsigned int person_index=0;person_index<people.size();person_index++) {
		//if this was seen in the last 3 frames
		cv::Scalar rect_color;
		if((frame_number-people[person_index].first_seen) < 5) rect_color = cv::Scalar(0,255,255);
		else if(frame_number-people[person_index].last_seen > 2) rect_color = cv::Scalar(0,0,255);
		else rect_color = cv::Scalar(0,255,0);

			std::stringstream text;
			text << "Person #"<<person_index;

			//draw Kalman estimates
			cv::Point estimate_origin = people[person_index].filter.bounding_rect_origin();

			cv::putText(*image,text.str(),estimate_origin,CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,255),kFontThickness,8);
			cv::rectangle(*image,people[person_index].filter.bounding_rect(),rect_color,1,8);
	}
}

void drawBags(const PersonList& people,const BagList& bags, cv::Mat* image, int frame_number) {
	for(unsigned int bag_index=0;bag_index<bags.size();bag_index++) {
		//if this was seen in the last 3 frames
		cv::Scalar rect_color;
		if(frame_number-bags[bag_index].first_seen < 5) rect_color = cv::Scalar(255,255,0);
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
			owner_center.x = people[owner_index].filter.bounding_rect().x+people[owner_index].filter.bounding_rect().width/2;
			owner_center.y = people[owner_index].filter.bounding_rect().y+people[owner_index].filter.bounding_rect().height/2;

			bag_center.x = bags[bag_index].filter.bounding_rect().x+bags[bag_index].filter.bounding_rect().width/2;
			bag_center.y = bags[bag_index].filter.bounding_rect().y+bags[bag_index].filter.bounding_rect().height/2;

			cv::line(*image,bag_center,owner_center,cv::Scalar(0,255,0),2,8);
	}
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
