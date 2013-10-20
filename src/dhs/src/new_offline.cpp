#include <fstream>
#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "utility.h"
#include "offline_event.cpp"

#define TYPE_BLANK 0


static const int kFontThickness = 2;
static const double kFontScale = 0.6;

struct descriptor {
	int event_name;
	cv::VideoCapture src;
	std::vector<int> start_times,end_times;
	int event_type;
	cv::Scalar first_upper,second_upper;
};

int trackbar_position=0;
/*
 *
 * Data file:
 *
 * offset from start
 * length of entire viewable sequence
 * first upper
 * first lower
 * second upper
 * second lower
 * lines for this video
 * event name
 * first upper
 * second upper
 * start time
 * end time
 * ...
 * start time
 * end time
 * lines for this video
 * ...
 */

static const std::string kVideo = "Video";

struct Helper {
	bool dirty;
	int position;
};

void OnMouse(int event,int x,int y,int flags, void* input) {
	if(event==CV_EVENT_LBUTTONDOWN) {
		Helper* data = (Helper*)input;
		data->position = (y-20)/20;
		data->dirty = true;
	}
}

void PlayPause(int event,int x,int y,int flags, void* input) {
	if(event==CV_EVENT_LBUTTONDOWN) {
		bool* play = (bool*)input;
		*play = !*play;
		if(*play) {
			cv::Mat controls(cv::Size(100,100),CV_8UC3,cv::Scalar(0));
			cv::rectangle(controls,cv::Rect(10,10,80,80),cv::Scalar(0,0,255),-1,4);
			imshow("Controls",controls);
		}
		else {
			cv::Mat controls(cv::Size(100,100),CV_8UC3,cv::Scalar(0));
			cv::circle(controls,cv::Point(50,50),40,cv::Scalar(0,255,0),-1,4);
			imshow("Controls",controls);
		}
	}
}

int main() {
	bool force_play=false;
	cv::namedWindow("Controls",CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Controls",65,190);
	cv::Mat controls(cv::Size(100,100),CV_8UC3,cv::Scalar(0));
	cv::rectangle(controls,cv::Rect(10,10,80,80),cv::Scalar(0,0,255),-1,4);
	imshow("Controls",controls);
	cv::setMouseCallback("Controls",PlayPause,&force_play);
	cv::namedWindow("Instances",CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Instances",65,370);
	Helper helper;
	helper.dirty = false;
	cv::setMouseCallback("Instances",OnMouse,&helper);
	//load the database
	std::ifstream fin;
	fin.clear();
	fin.open("offline_database/database.txt");
	std::vector<descriptor> all_descriptors;

	int offset=0,length=1;
	uint search_code;
	fin >>offset >>length;

	utility::ColorSingleListType bag_colors;
	bag_colors.push_back(cv::Scalar(0));
	utility::ColorPairListType people_colors;

	cv::Scalar first_upper,first_lower,second_upper,second_lower;

	int hue,sat,val;
	fin >> hue >> sat >> val;
	first_upper = cv::Scalar(hue,sat,val);
	fin >> hue >> sat >> val;
	first_lower = cv::Scalar(hue,sat,val);

	fin >> hue >> sat >> val;
	second_upper = cv::Scalar(hue,sat,val);
	fin >> hue >> sat >> val;
	second_lower = cv::Scalar(hue,sat,val);

	utility::Pair_<cv::Scalar> first_color,second_color;
	first_color.values[0] = first_upper;
	first_color.values[1] = first_lower;
	second_color.values[0] = second_upper;
	second_color.values[1] = second_lower;

	people_colors.push_back(first_color);
	people_colors.push_back(second_color);

	int lines;
	fin >> lines;

	int video_index=1;

	descriptor first;
	first.src.open("offline_database/blank.avi");
	first.event_type = TYPE_BLANK;
	all_descriptors.push_back(first);

	while(fin.good()) {
		descriptor temp;
		int hue,sat,val;
		//get the event name
		fin >> temp.event_name;

		//colors
		fin >> hue >> sat >> val;
		temp.first_upper = cv::Scalar(hue,sat,val);
		fin >> hue >> sat >> val;
		temp.second_upper = cv::Scalar(hue,sat,val);

		for(int i=0;i<lines;i++) {
			//get start/end times
			int temp_time;
			fin >> temp_time;
			temp.start_times.push_back(temp_time);
			fin >> temp_time;
			temp.end_times.push_back(temp_time);
		}
		std::stringstream name;
		name<<"offline_database/"<<video_index++<<".avi";
		temp.src.open(name.str());

		first_color.values[0] = first_upper;
		first_color.values[1] = first_lower;
		second_color.values[0] = second_upper;

		all_descriptors.push_back(temp);
		fin>>lines;
	}

	//show the video window
	int trackbar_position = 0;
	cv::namedWindow(kVideo,CV_WINDOW_AUTOSIZE);
	cv::moveWindow(kVideo,575,100);
	//load first video
	cv::Mat output_frame;
	all_descriptors[TYPE_BLANK].src >> output_frame;
	imshow(kVideo,output_frame);
	//trackbar
	cv::createTrackbar("Position",kVideo,&trackbar_position,length-offset);
	cv::waitKey(1);

	search_code = MenuInitial(bag_colors,people_colors);

	//create the menu
	int last_trackbar_position = trackbar_position;
	bool move_bar = true;
	bool playing = false;
	int stop_at = 1;
	bool first_time = true;
	cv::VideoCapture* current_source;
	while(true) {
		if(trackbar_position>=length) {
			search_code = MenuInitial(bag_colors,people_colors);
			move_bar = true;
		}
		int event = DecodeEvent(search_code);
		int person_one_event_index = DecodePersonOne(search_code);
		int person_two_event_index = DecodePersonTwo(search_code);

		//find the event
		int event_index=-1;
		for(int i=1;i<all_descriptors.size();i++) {
			if(all_descriptors[i].event_name==event) {
				if(all_descriptors[i].first_upper==people_colors[person_one_event_index][0]) {
					if(person_two_event_index==EVENT_NOT_USED) {
						event_index=i;
					}
					else if(all_descriptors[i].second_upper==people_colors[person_two_event_index][0]) {
						event_index=i;
					}
				}
			}
		}
		if(move_bar) {
			trackbar_position = all_descriptors[event_index].start_times[0]+1;
			cv::setTrackbarPos("Position",kVideo,trackbar_position);
			move_bar = false;

		}
		if(event_index!=-1) {
			//populate instances box
			cv::Mat instances_mat(cv::Size(350,all_descriptors[event_index].start_times.size()*20+20),CV_8UC1,cv::Scalar(0));
			cv::Point top_left(5,0);
			MakeButton(&instances_mat,"The event is seen at these times. Click one to view",top_left, cv::Scalar(0));
			top_left.y+=20;
			for(int i=0;i<all_descriptors[event_index].start_times.size();i++) {
				std::stringstream name;
				name<<(all_descriptors[event_index].start_times[i]+15)/30<<" seconds - "<<(all_descriptors[event_index].end_times[i]+15)/30<<" seconds";
				MakeButton(&instances_mat,name.str(),top_left, cv::Scalar(0));
				top_left.y+=20;
			}
			imshow("Instances",instances_mat);

			//show the video
			if(trackbar_position!=last_trackbar_position) {
				//update the video frame
				//check if the frame should be grabbed from the blank or not
				for(int i=0;i<all_descriptors[event_index].start_times.size();i++) {
					if(trackbar_position > all_descriptors[event_index].start_times[i]) {
						if(trackbar_position < all_descriptors[event_index].end_times[i]) {
							//show from analysis
							all_descriptors[event_index].src.set(CV_CAP_PROP_POS_FRAMES,trackbar_position);
							all_descriptors[event_index].src >> output_frame;
							current_source = &all_descriptors[event_index].src;
							if(!first_time) playing = true;
							first_time = false;
							stop_at = all_descriptors[event_index].end_times[i];
							break;
						}
						else {
							//show from blank
							all_descriptors[TYPE_BLANK].src.set(CV_CAP_PROP_POS_FRAMES,trackbar_position);
							all_descriptors[TYPE_BLANK].src >> output_frame;
							current_source = &all_descriptors[TYPE_BLANK].src;
						}
					}
					else {
						//show from blank
						all_descriptors[TYPE_BLANK].src.set(CV_CAP_PROP_POS_FRAMES,trackbar_position);
						all_descriptors[TYPE_BLANK].src >> output_frame;
						current_source = &all_descriptors[event_index].src;
					}
				}

				last_trackbar_position = trackbar_position;
			}
		}
		if(helper.dirty) {
			trackbar_position = all_descriptors[event_index].start_times[helper.position]+1;
			cv::setTrackbarPos("Position",kVideo,trackbar_position);
			helper.dirty = false;
		}
		else if(playing || force_play ) {
			if(trackbar_position < stop_at || (force_play&&trackbar_position<length)) {
				last_trackbar_position = ++trackbar_position;
				(*current_source) >> output_frame;
				cv::setTrackbarPos("Position",kVideo,trackbar_position);
			}
			else {
				playing = false;

			}
		}

		std::stringstream time;
		time<<trackbar_position/30.;
		std::string time_out = time.str();
		while(time_out.length()>4) {
			time_out.erase(time_out.begin()+4);
		}
		cv::putText(output_frame,time_out,cv::Point(5,470),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255,255,255),kFontThickness,8);
		cv::putText(output_frame,"seconds",cv::Point(55,470),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255,255,255),kFontThickness,8);

		imshow(kVideo,output_frame);
		cv::waitKey(33);
	}


}
