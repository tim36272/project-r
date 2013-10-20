#include <ros/ros.h>
#include "MessageFetcher.h"
#include "Analysis.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utility.h"
#include "Types.h"
#include "InstanceGrabber.h"
#include "event.cpp"
#include "instancer_setup.cpp"

static const std::string kWindow = "Live";
static const bool kRecord = true;
static const int kFontThickness = 2;
static const double kFontScale = 0.6;

void drawPeople(const PersonList& people,cv::Mat* image, int frame_number);
void drawBags(const PersonList& people,const BagList& bags, cv::Mat* image, int frame_number);
int getVideoNumber(const char path[]);
void DrawInstancerLabels(const std::vector<InstanceGrabber>& instances,cv::Mat* image);
void ShowGuy(const cv::Scalar& first_upper_color,const cv::Scalar& first_lower_color, const cv::Scalar& second_upper_color,const cv::Scalar& second_lower_color,cv::Mat* image);

int main(int argc, char** argv)
{

	ros::init(argc, argv, "live");

	MessageFetcher ros_handle;
	cv::Mat color_raw,depth_raw,hsv_raw;
	Analysis analysis_handle("/r/people","/r/bags");
	PersonList people;
	BagList bags;
	bool run = true;
	int frame_number;
	std::vector<InstanceGrabber> instances;
	
	//create output writer
	cv::VideoWriter writer;
	if(kRecord) {
		std::stringstream video_output_name;
		video_output_name << "saves/save_"<<getVideoNumber("saves/video_number")<<".avi";
		writer.open(video_output_name.str(),CV_FOURCC('D','I','V','X'),23,cv::Size(640,480),true);
	}

	//ask for colors
	cv::Mat pre_menu(cv::Size(kMenuWidth,kMenuItemHeight),CV_8UC3,cv::Scalar(0));
	MakeButton(&pre_menu,"Please upload colors then press any key",cv::Point(0,0),cv::Scalar(0));
	imshow(kMenuName,pre_menu);
	while(cv::waitKey(1)==-1) ros::spinOnce();

	//get event to detect
	uint search_code = MenuInitial(analysis_handle.bag_colors_,analysis_handle.people_colors_);

	//decide how many instances to create and add an instance for each of them
	int event = DecodeEvent(search_code);
	int person_one_event_index = DecodePersonOne(search_code);
	int person_two_event_index = DecodePersonTwo(search_code);
	int bag_event_index = DecodeBag(search_code);

	SetupPossabilities(event,
					   person_one_event_index,
					   person_two_event_index,
					   bag_event_index,
					   analysis_handle.people_colors_,
					   analysis_handle.bag_colors_,
					   &instances);
	if(instances.size()==0) {
		std::cout<<"There were no color schemes meeting that criteria, no events will be detected"<<std::endl;
	}

	cv::namedWindow(kWindow,CV_WINDOW_AUTOSIZE);
	cv::moveWindow(kWindow,15,200);
	cv::Mat initial(cv::Size(300,20),CV_8UC1,cv::Scalar(0));
	MakeButton(&initial,"Ready to receive video",cv::Point(0,0),cv::Scalar(0,0,0));
	imshow(kWindow,initial);

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

		//look for the desired colors
		frame_number = analysis_handle.Update(color_raw,depth_raw,&people,&bags);

		//update all the instancer members
		for(int instance_index=0;instance_index<instances.size();instance_index++) {
			instances[instance_index].Update(frame_number,people,bags);
		}

		//show the blobs on the output frame
		cv::Mat output;
		color_raw.copyTo(output);

		drawPeople(people,&output,frame_number);
		drawBags(people,bags,&output,frame_number);
		DrawInstancerLabels(instances,&output);

		if(instances.size()>0) {
			ShowGuy(instances[0].person_one_upper_color_,
					instances[0].person_one_lower_color_,
					instances[0].person_two_upper_color_,
					instances[0].person_two_lower_color_,
					&output);
		}

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
	for(uint instance_index=0;instance_index<instances.size();instance_index++) {
		std::cout<<instances[instance_index]<<std::endl;
		std::cout<<"------------------------"<<std::endl;
	}

	return 0;
}

void drawPeople(const PersonList& people,cv::Mat* image, int frame_number) {
	for(unsigned int person_index=0;person_index<people.size();person_index++) {
		//if this was seen in the last 3 frames
		cv::Scalar rect_color;
		int depth = people[person_index].depth_position;
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
			int owner_index = /*bags[bag_index].belongs_to;*/0;
			if(owner_index!=-1 && (int(people.size())-1)>=owner_index) {
				if(utility::AreClose(people[owner_index].filter.bounding_rect(),bags[bag_index].filter.bounding_rect()) ) {
					owner_center.x = people[owner_index].filter.bounding_rect().x+people[owner_index].filter.bounding_rect().width/2;
					owner_center.y = people[owner_index].filter.bounding_rect().y+people[owner_index].filter.bounding_rect().height/2;

					bag_center.x = bags[bag_index].filter.bounding_rect().x+bags[bag_index].filter.bounding_rect().width/2;
					bag_center.y = bags[bag_index].filter.bounding_rect().y+bags[bag_index].filter.bounding_rect().height/2;
					cv::line(*image,bag_center,owner_center,cv::Scalar(0,255,0),2,8);
				}
			}
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

void DrawInstancerLabels(const std::vector<InstanceGrabber>& instances,cv::Mat* image) {
	int x_origin=5;
	int y_origin=15;
	for(uint instance_index=0;instance_index<instances.size();instance_index++) {
		//draw initial label

		std::stringstream label;
			switch(instances[instance_index].event_code()) {
			case EVENT_ONE_PERSON_SEARCH:
				label <<"Person #"
					  <<instances[instance_index].person_one_source()
					  <<" is visible";
				break;
			case EVENT_ONE_PERSON_ABANDON:
/*				label <<"Person #"
					  <<instances[instance_index].person_one_source()
					  <<" has abandoned bag #"
					  <<instances[instance_index].bag_source();*/
				label <<"Person # 0 has abandoned bag #0";
				break;
			case EVENT_TWO_PERSON_SEARCH:
				label <<"Person #"
					  <<instances[instance_index].person_one_source()
					  <<" and person #"
					  <<instances[instance_index].person_two_source()
					  <<" are meeting";
				break;
			case EVENT_TWO_PERSON_STEAL:
/*				label <<"Person #"
					  <<instances[instance_index].person_two_source()
					  <<" could be stealing bag #"
					  <<instances[instance_index].bag_source()
					  <<" from person #"
					  <<instances[instance_index].person_one_source();*/
				label <<"Person # 1 could be stealing bag #0 from person #0";
				break;
			case EVENT_TWO_PERSON_EXCHANGE:
				label <<"Person #"
					  <<instances[instance_index].person_one_source()
					  <<" and person #"
					  <<instances[instance_index].person_two_source()
					  <<" are exchanging bag #"
					  <<instances[instance_index].bag_source();


				break;
			default:
				//Incorrect objective code
				assert(false);
				break;
			}

		if( instances[instance_index].EventInProgress() ) {
			cv::putText(*image,label.str(),cv::Point(x_origin,y_origin),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,255,0),kFontThickness,8);
		}
		y_origin+=20;
	}
}

void ShowGuy(const cv::Scalar& first_upper_color,const cv::Scalar& first_lower_color, const cv::Scalar& second_upper_color,const cv::Scalar& second_lower_color,cv::Mat* image) {
	cv::Rect upper(560,440,39,20);
	cv::Rect lower(560,460,39,20);
	cv::Rect text(560,420,39,20);

	if(second_upper_color==cv::Scalar(0)) {
		upper.x+=40;
		lower.x+=40;
		text.x+=40;
	}

	cv::rectangle(*image,upper,utility::HSV2BGR(first_upper_color),-1,4);
	cv::rectangle(*image,lower,utility::HSV2BGR(first_lower_color),-1,4);
	cv::rectangle(*image,text,cv::Scalar(255,255,255),-1,4);
	cv::putText(*image,"0",upper.tl()+cv::Point(15,-5),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,0),kFontThickness,8);
	upper.x+=40;
	lower.x+=40;
	text.x+=40;

	if(second_upper_color!=cv::Scalar(0)) {
		cv::rectangle(*image,upper,utility::HSV2BGR(second_upper_color),-1,4);
		cv::rectangle(*image,lower,utility::HSV2BGR(second_lower_color),-1,4);
		cv::rectangle(*image,text,cv::Scalar(255,255,255),-1,4);
		cv::putText(*image,"1",upper.tl()+cv::Point(15,-5),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(0,0,0),kFontThickness,8);
	}



}
