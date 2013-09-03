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
#include "InstanceGrabber.h"
#include "Analysis.h"

#define NO_FRAMES_RECEIVED -2
#define BAD_INPUT -3

//static const std::string kWindow = "output";
static const int kFontThickness = 1;
static const double kFontScale = 0.6;

void GetVideo(MessageFetcher& ros_handle, std::vector<cv::Mat>* frames);
void VideoPositionCallback(int,void*);
void ButtonCallback(int state,void*);
void ShowMainWindow(int* position,std::vector<cv::Mat>& frames);
void SetupEventPicker(bool* one_person_search, bool* two_person_search, bool* bag_unattended, bool* bag_stolen, bool* bag_exchanged, bool* go);
void ShowClips(const std::vector<utility::Pair_<uint> >& instances,const std::vector<cv::Mat>& frames);
void onMouseInstances(int event,int x,int y,int flags, void* data);

struct helper {
	bool one_person_search,
		 two_person_search,
		 bag_unattended,
		 bag_stolen,
		 bag_exchanged,
		 go;
	helper() {
		one_person_search=false,
		 two_person_search=false,
		 bag_unattended=false,
		 bag_stolen=false,
		 bag_exchanged=false,
		 go=false;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imageProcessor");

	MessageFetcher ros_handle;
	OfflineAnalysis colors_handle("/r/people","/r/bags");
	Analysis analysis_handle("/r/people","/r/bags");
	
	bool run = true;
	int frame_number;

	cv::Mat temp(cv::Size(20,20),CV_8UC1,cv::Scalar(0));
	imshow("temp",temp);

	//event picker
	helper options;

	SetupEventPicker(&options.one_person_search, &options.two_person_search, &options.bag_unattended, &options.bag_stolen,  &options.bag_exchanged, &options.go);

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
	while(options.go==false) { if(cv::waitKey(1)=='q') break; }

	//be sure we have enough colors
	bool need_one_person=false,need_two_people=false,need_bag=false;
	if(options.bag_unattended || options.bag_exchanged || options.bag_stolen) {
		if(colors_handle.bag_colors_.size()<1) {
			need_bag = true;
		}
	}
	if(options.one_person_search || options.bag_unattended) {
		if(colors_handle.people_colors_.size()<1) {
			need_one_person=true;
		}
	}
	else { //two people
		if(colors_handle.people_colors_.size()==0) {
			need_one_person = true;
		}
		else if (colors_handle.people_colors_.size()==1) {
			need_two_people = true;
		}
	}

	if(need_one_person) {
		std::cout<<"Please add a person's color distribution"<<std::endl;
		while(colors_handle.people_colors_.size()<1) ros::spinOnce();
	}
	if(need_two_people) {
		std::cout<<"Please add two more peoples' color distributions"<<std::endl;
		while(colors_handle.people_colors_.size()<2) ros::spinOnce();
	}
	if(need_bag) {
		std::cout<<"Please add a bag color distribution"<<std::endl;
		while(colors_handle.bag_colors_.size()<1) ros::spinOnce();
	}

	//get the colors
	cv::Scalar first_upper = colors_handle.people_colors_[0][0],
			   first_lower = colors_handle.people_colors_[0][1],
			   second_upper,second_lower,bag;
	if(options.two_person_search || options.bag_exchanged || options.bag_stolen) {
		second_upper = colors_handle.people_colors_[1][0];
		second_lower = colors_handle.people_colors_[1][1];
	}
	if(options.bag_stolen || options.bag_exchanged || options.bag_unattended) {
		bag = colors_handle.bag_colors_[0];
	}

	//ready to do analysis
	InstanceGrabber instancer;
	if(options.one_person_search) {
		std::cout<<"Setup as one person track"<<std::endl;
		instancer.Setup(R_ONE_PERSON_TRACK,first_upper,first_lower);
	}
	else if(options.two_person_search) {
		instancer.Setup(R_TWO_PERSON_MEET,first_upper,first_lower,second_upper,second_lower);
	}
	else if(options.bag_exchanged) {
		instancer.Setup(R_TWO_PERSON_BAG_EXCHANGE,first_upper,first_lower,second_upper,second_lower,bag);
	}
	else if(options.bag_stolen) {
		instancer.Setup(R_TWO_PERSON_BAG_STEAL,first_upper,first_lower,second_upper,second_lower,bag);
	}
	else if(options.bag_unattended) {
		instancer.Setup(R_ONE_PERSON_BAG_UNATTENDED,first_upper,first_lower,bag);
	}

	//analysis handle
	PersonList people;
	BagList bags;

	//main processing loop
	std::cout<<"Processjng..."<<std::endl;
	for(uint frame_index=0;frame_index<frames.size();frame_index++) {
		analysis_handle.Update(frames[frame_index],cv::Mat(),&people,&bags);

		instancer.Update(frame_index,people,bags);
	}
	std::cout<<"Done processing"<<std::endl;
	std::cout<<"instancer size: "<<instancer.instances_.size();
	instancer.PrintInstances(std::cout);
	ShowClips(instancer.Instances(),frames);
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

void ShowClips(const std::vector<utility::Pair_<uint> >& instances,const std::vector<cv::Mat>& frames) {
	//generate window with instances
	cv::Size control_size(400,instances.size()*20);
	cv::Mat control_window(control_size,CV_8UC1,cv::Scalar(0));

	for(uint instance_index=0;instance_index<instances.size();instance_index++) {
		std::stringstream name;
		name<<"Start: "<<instances[instance_index][0]<<" End: "<<instances[instance_index][1];
		cv::putText(control_window,name.str(),cv::Point(30,instance_index*20+15),CV_FONT_HERSHEY_SIMPLEX,kFontScale,cv::Scalar(255),kFontThickness,8);
		cv::rectangle(control_window,cv::Rect(5,instance_index*20+3,14,14),cv::Scalar(255),-1,8);
		cv::rectangle(control_window,cv::Rect(7,instance_index*20+5,10,10),cv::Scalar(128),-1,8);
	}
	int active_instance=0,last_active_instance=-1;
	imshow("Instances",control_window);
	cv::setMouseCallback("Instances",onMouseInstances,&active_instance);

	int slider_min=0,slider_max=1;

	while(cv::waitKey(1)!='q') {
		if(last_active_instance!=active_instance) {
			//load the new instance
			slider_min = instances[active_instance][0];
			slider_max = instances[active_instance][1];
			last_active_instance = active_instance;
			cv::setTrackbarPos("Position","Video",slider_min);
			imshow("Video",frames[slider_min]);
		}
		int current = cv::getTrackbarPos("Position","Video");

		if(current < slider_min) {
			cv::setTrackbarPos("Position","Video",slider_min);
		}
		else if(current > slider_max) {
			cv::setTrackbarPos("Position","Video",slider_max);
		}

	}

}

void onMouseInstances(int event,int x,int y,int flags, void* data) {
	int* active_instance = (int*)data;
	if(event==CV_EVENT_LBUTTONDOWN) {
		*active_instance = y/20;
		std::cout<<"Active instance: "<<*active_instance<<std::endl;
	}
}
