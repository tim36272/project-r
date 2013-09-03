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
#define BAD_INPUT -3

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

void GetInstancesOfOnePersonBagStolen(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& bag, std::vector<std::vector<int> >* instances);
void GetInstancesOfOnePersonBagUnattended(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower, std::vector<std::vector<int> >* instances);
void GetInstancesOfOnePerson(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower, std::vector<std::vector<int> >* instances);
void GetInstancesOfTwoPeopleBagExchanged(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& second_upper,const cv::Scalar& second_lower,const cv::Scalar& bag, std::vector<std::vector<int> >* instances);
void GetInstancesOfTwoPeople(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& second_upper,const cv::Scalar& second_lower, std::vector<std::vector<int> >* instances);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imageProcessor");

	MessageFetcher ros_handle;
	OfflineAnalysis colors_handle("/r/people","/r/bags");
	
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

	//validate input
	if(event==ONE_PERSON_SEARCH) {
		if(second_event==BAG_EXCHANGED) {
			std::cout<<"That search is not possible"<<std::endl;
			return BAD_INPUT;
		}
	}
	else {
		if(second_event==BAG_UNATTENDED || second_event==BAG_STOLEN) {
			std::cout<<"That search is not possible"<<std::endl;
			return BAD_INPUT;
		}
	}

	int first_upper_hue=0,first_lower_hue=0,second_upper_hue=0,second_lower_hue=0,bag_hue=0;

	//be sure we have enough colors
	bool need_one_person=false,need_two_people=false,need_bag=false;
	if(second_event!=0) {
		if(colors_handle.bag_colors_.size()<1) {
			need_bag = true;
		}
	}
	if(event==ONE_PERSON_SEARCH) {
		if(colors_handle.people_colors_.size()<1) {
			need_one_person=true;
		}
	}
	else {
		if(colors_handle.people_colors_.size()==0) {
			need_one_person = true;
		}
		else if (colors_handle.people_colors_.size()==1) {
			need_two_people = true;
		}
	}

	if(need_one_person) {
		std::cout<<"Please add a person's color distribution"<<std::endl;
		while(colors_handle.people_colors_.size()<1) continue;
	}
	if(need_two_people) {
		std::cout<<"Please add two more peoples' color distributions"<<std::endl;
		while(colors_handle.people_colors_.size()<2) continue;
	}
	if(need_bag) {
		std::cout<<"Please add a bag color distribution"<<std::endl;
		while(colors_handle.bag_colors_.size()<1) continue;
	}

	//get the colors
	cv::Scalar first_upper = colors_handle.people_colors_[0][0],
			   first_lower = colors_handle.people_colors_[0][1],
			   second_upper,second_lower,bag;
	if(event==TWO_PERSON_SEARCH) {
		second_upper = colors_handle.people_colors_[1][0];
		second_lower = colors_handle.people_colors_[1][1];
	}
	if(second_event!=0) {
		bag = colors_handle.bag_colors_[0];
	}

	//ready to do analysis
	std::vector<std::vector<int> > instances;
	if(event==ONE_PERSON_SEARCH) {
		if(second_event==BAG_STOLEN) {
			GetInstancesOfOnePersonBagStolen(frames,first_upper,first_lower,bag,&instances);
		}
		else if(second_event==BAG_UNATTENDED) {
			GetInstancesOfOnePersonBagUnattended(frames,first_upper,first_lower,&instances);
		}
		else {
			GetInstancesOfOnePerson(frames,first_upper,first_lower,&instances);
		}
	}
	else { //TWO_PERSON_SEARCH
		if(second_event==BAG_EXCHANGED) {
			GetInstancesOfTwoPeopleBagExchanged(frames,first_upper,first_lower,second_upper,second_lower,bag,&instances);
		}
		else {
			GetInstancesOfTwoPeople(frames,first_upper,first_lower,second_upper,second_lower,&instances);
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

void GetInstancesOfOnePersonBagStolen(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& bag, std::vector<std::vector<int> >* instances) {
/*	//make analysis handle and data
	Analysis analysis_handle("/r/people","/r/bags");
	PersonList people,full_people_list;
	BagList bags,full_bag_list;

	//get instances of the people and bag
	for(uint frame_index=0;frame_index<frames;frame_index++) {
		analysis_handle.Update(frames[frame_index],cv::Mat(),&people,&bags);
		//check each person and see if they have disappeared. If so, add them to the permanent list
		for(uint person_index=0;person_index<people.size();person_index++) {
			if((frame_index - people[person_index].last_seen)>30) {
				//if they haven't been seen in more than 30 frames (1 second)
				full_people_list.push_back(people[person_index]);
				people.erase(people.begin()+person_index);
			}
		}
	}
*/
	//figure out if/when the bag belongs to the person
}
void GetInstancesOfOnePersonBagUnattended(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower, std::vector<std::vector<int> >* instances) {

}
void GetInstancesOfOnePerson(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower, std::vector<std::vector<int> >* instances) {

}
void GetInstancesOfTwoPeopleBagExchanged(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& second_upper,const cv::Scalar& second_lower,const cv::Scalar& bag, std::vector<std::vector<int> >* instances) {

}
void GetInstancesOfTwoPeople(const std::vector<cv::Mat> frames, const cv::Scalar& first_upper,const cv::Scalar& first_lower,const cv::Scalar& second_upper,const cv::Scalar& second_lower, std::vector<std::vector<int> >* instances) {

}
