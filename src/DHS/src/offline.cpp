#include <ros/ros.h>
#include "MessageFetcher.h"
#include "OfflineAnalysis.h"
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include "utility.h"
#include <vector>
#include "Types.h"
#include "InstanceGrabber.h"
#include "event.cpp"
#include "instancer_setup.cpp"
#include "Analysis.h"

// ROS BAG Interface
#include "rosbag/bag.h"
#include "rosbag/view.h"

#define NO_FRAMES_RECEIVED -2
#define BAD_INPUT -3

//static const std::string kWindow = "output";
static const int kFontThickness = 1;
static const double kFontScale = 0.6;

bool SetupVideo(rosbag::Bag* bag_handle,const std::string& bag_name);
void VideoPositionCallback(int,void*);
void ShowMainWindow(int* position,std::vector<cv::Mat>& frames);
void ShowClips(const std::vector<utility::Pair_<uint> >& instances,const std::vector<cv::Mat>& frames);
void onMouseInstances(int event,int x,int y,int flags, void* data);


int main(int argc, char** argv) {
	std::string bag_name;
	if(argc <=2) {
		std::cout<<"Usage: rosrun dhs offline bag_name [start offset]"<<std::endl;
		return 0;
	}
	else {
		bag_name = argv[1];
	}
	ros::init(argc, argv, "imageProcessor");

	MessageFetcher ros_handle;
	Analysis analysis_handle("/r/people","/r/bags");
	std::vector<InstanceGrabber> instances;
	
	rosbag::Bag bag_handle;

	cv::Mat temp(cv::Size(20,20),CV_8UC1,cv::Scalar(0));
	imshow("temp",temp);

	//ask for colors
	cv::Mat pre_menu(cv::Size(kMenuWidth,kMenuItemHeight),CV_8UC3,cv::Scalar(0));
	MakeButton(&pre_menu,"Please upload colors then press any key",cv::Point(0,0),cv::Scalar(0));
	imshow(kMenuName,pre_menu);
	while(cv::waitKey(1)==-1) ros::spinOnce();

	//get event to detect
	uint search_code = MenuInitial(analysis_handle.bag_colors_,analysis_handle.colors_);

	//decide how many instances to create
	int event = DecodeEvent(search_code);
	int person_one_event_index = DecodePersonOne(search_code);
	int person_two_event_index = DecodePersonTwo(search_code);
	int bag_event_index = DecodeBag(search_code);

	SetupPossabilities(event,
					   person_one_event_index,
					   person_two_event_index,
					   bag_event_index,
					   analysis_handle.colors_,
					   analysis_handle.bag_colors_,
					   &instances);
	if(instances.size()==0) {
		std::cout<<"There were no color schemes meeting that criteria, no events will be detected"<<std::endl;
	}

	//get the video from the bag
	SetupVideo(&bag_handle,bag_name);


	//analysis handle
	PersonList people;
	BagList bags;

	  rosbag::View::iterator bag_depth_iterator, bag_color_iterator;
	  rosbag::View::iterator bag_depth_iterator_end, bag_color_iterator_end;

	  rosbag::View rgbView(   bag_handle, rosbag::TopicQuery("/camera/rgb/image_color") );
	  rosbag::View depthView( bag_handle, rosbag::TopicQuery("/camera/depth_registered/image_raw") );

	  bag_depth_iterator_end = depthView.end();
	  bag_depth_iterator = depthView.begin();
	  bag_color_iterator_end = rgbView.end();
	  bag_color_iterator = rgbView.begin();

	//main processing loop
	std::cout<<"Processing..."<<std::endl;
	bool running=true;
	int frame_number=0;
	int frames_to_skip = (argc==3) ? atoi(argv[2]) : 0;
	//skip ahead in the video, possible 0 frames
	for(int i=0;i<frames_to_skip;i++) {
		bag_depth_iterator++;
		bag_color_iterator++;
	}
	while(running && ros::ok()){
		//get a frame from the bag
		rosbag::MessageInstance rgbMessageInst   = *bag_color_iterator;
		rosbag::MessageInstance depthMessageInst = *bag_depth_iterator;

		// convert to sensor_msgs/Image
		sensor_msgs::ImageConstPtr rgbMessage = rgbMessageInst.instantiate<sensor_msgs::Image>();
		sensor_msgs::ImageConstPtr depthMessage = depthMessageInst.instantiate<sensor_msgs::Image>();

		if( rgbMessage == NULL || depthMessage == NULL )  {
		  std::cout << "Casting Error, exiting" << std::endl;
		  running = false;
		}

		// convert sensor_msgs/Imag to cv::imagePtr <><> this is the opencv bridge
		cv_bridge::CvImagePtr cv_depthPtr, cv_rgbPtr;
		try
		{
		  cv_rgbPtr   = cv_bridge::toCvCopy( rgbMessage );
		  cv_depthPtr = cv_bridge::toCvCopy( depthMessage);
		}
		catch(cv_bridge::Exception &e )
		{
		  std::cout << "Error converting ros message" << std::endl << "   What: ";
		  std::cout << e.what() << std::endl;
		  running = false;
		}
		//convert depth
		cv_depthPtr->image.convertTo(cv_depthPtr->image,CV_8UC1,255./8000.);

		analysis_handle.Update(cv_rgbPtr->image,cv_depthPtr->image,&people,&bags);
		for(uint instance_index=0;instance_index<instances.size();instance_index++) {
			instances[instance_index].Update(frame_number,people,bags);
		}
		bag_depth_iterator++;
		bag_color_iterator++;

	    if( ( bag_depth_iterator == bag_depth_iterator_end ) ||
	        ( bag_color_iterator == bag_color_iterator_end )    ) {
	      running = false;
	      break;
	    }
		frame_number++;
		if(frame_number%10==0) std::cout<<"Progress: "<<double(frame_number+frames_to_skip)/rgbView.size()<<std::endl;
	}
	//print all remaining instances
	for(uint instance_type_index=0;instance_type_index<instances.size();instance_type_index++) {
		std::cout<<"instance type "<<instance_type_index<<" : "<<instances[instance_type_index]<<std::endl;
	}
	std::cout<<"Done processing"<<std::endl;

	//cull instances list
	//merge close instances
	for(uint instance_type_index=0;instance_type_index<instances.size();instance_type_index++) {
		for(uint instance_index=0;instance_index<instances[instance_type_index].instances_.size()-1;instance_index++) {
			int time_between_instances = instances[instance_type_index].instances_[instance_index+1][0]-instances[instance_type_index].instances_[instance_index][1];
			if(time_between_instances < 100) {
				//probably dropped frames between
				int new_end = instances[instance_type_index].instances_[instance_index+1][1];
				instances[instance_type_index].instances_[instance_index][1] = new_end;
				instances[instance_type_index].instances_.erase(instances[instance_type_index].instances_.begin()+instance_index+1);
			}
		}

	}
	//remove short instances
	for(uint instance_type_index=0;instance_type_index<instances.size();instance_type_index++) {
		for(uint instance_index=0;instance_index<instances[instance_type_index].instances_.size();instance_index++) {
			int length_of_instance = instances[instance_type_index].instances_[instance_index][1]-instances[instance_type_index].instances_[instance_index][0];
			if(length_of_instance < 10) {
				//probably false positive
				instances[instance_type_index].instances_.erase(instances[instance_type_index].instances_.begin()+instance_index);
			}
		}

	}

	//print all remaining instances
	for(uint instance_type_index=0;instance_type_index<instances.size();instance_type_index++) {
		std::cout<<instances[instance_type_index]<<std::endl;
	}

	std::ofstream fout;
	fout.clear();
	fout.open("offline/data.txt");
	//save clips
	//for each type of instance, read the bag file again to get the video and annotate it
	for(uint instance_type_index=0;instance_type_index<instances.size();instance_type_index++) {
		  //for each instance make a clip
		  for(uint instance_index=0;instance_index<instances[instance_type_index].instances_.size();instance_index++) {
			  bag_color_iterator = rgbView.begin();
			  for(int i=0;i<instances[instance_type_index].instances_[instance_index][0];i++) {
				  bag_color_iterator++;
			  }

				cv::VideoWriter writer;
				std::stringstream video_output_name;
				video_output_name << "offline/"<<instance_type_index
								  <<"_"<<instance_index<<"_"
								  <<instances[instance_type_index].event_code()
								  <<"_"<<instances[instance_type_index].person_one_source_index_
								  <<"_"<<instances[instance_type_index].person_two_source_index_
								  <<"_"<<instances[instance_type_index].bag_source_index_
								  <<".avi";
				writer.open(video_output_name.str(),CV_FOURCC('D','I','V','X'),30,cv::Size(640,480),true);
				fout<<video_output_name.str()<<std::endl;

			  int frames = instances[instance_type_index].instances_[instance_index][1] - instances[instance_type_index].instances_[instance_index][0];
			  for(int frame_index=0;frame_index<frames;frame_index++) {
				rosbag::MessageInstance rgbMessageInst   = *bag_color_iterator;

				// convert to sensor_msgs/Image
				sensor_msgs::ImageConstPtr rgbMessage = rgbMessageInst.instantiate<sensor_msgs::Image>();

				if( rgbMessage == NULL )  {
				  std::cout << "Casting Error, exiting" << std::endl;
				  running = false;
				}

				// convert sensor_msgs/Imag to cv::imagePtr <><> this is the opencv bridge
				cv_bridge::CvImagePtr cv_rgbPtr;
				try
				{
				  cv_rgbPtr   = cv_bridge::toCvCopy( rgbMessage );
				}
				catch(cv_bridge::Exception &e )
				{
				  std::cout << "Error converting ros message" << std::endl << "   What: ";
				  std::cout << e.what() << std::endl;
				  running = false;
				}

				writer.write(cv_rgbPtr->image);
				bag_color_iterator++;

			  }
			  writer.release();
		  }
	}

///	ShowClips(instancer.Instances(),frames);

	//cleanup
	cv::destroyAllWindows();
	return 0;
}



bool SetupVideo(rosbag::Bag* bag_handle,const std::string& bag_name) {
	cv::Mat color_raw,depth_raw;

	// try to open the bag file
	  try
	  {
	    bag_handle->open(bag_name, rosbag::bagmode::Read);
	  }
	  catch( rosbag::BagException& exception)
	  {
	    std::cout << "Could not open output file" << std::endl;
	    std::cout << "What: " << exception.what() << std::endl;
	    return false;
	  }
	  // Note: rosbag is a read write syncronizer interface but
	  //          the reader portion is only accessable through
	  //          the rosbag::view class, think of the view as a search view

	  rosbag::View rosbagView( *bag_handle ); // First look at the bag with no critira

	  ros::Time startTime = rosbagView.getBeginTime();
	  ros::Time endTime   = rosbagView.getEndTime();
	  ros::Duration movieLength = endTime - startTime;
	  uint32_t  numberOfImages = rosbagView.size();

	  std::cout << "Ros::bag has " << numberOfImages << " message and is ";
	  std::cout << movieLength.toSec() << " second(s) long" << std::endl;


	  std::vector<const rosbag::ConnectionInfo*> connections = rosbagView.getConnections();

	  std::cout << "  and contains the following messages:" << std::endl;
	  for( int i=0; i<connections.size(); ++i)
	  {
		  std::cout << "    Message[" << i << "]: Name - \"";
		  std::cout << connections[i]->topic << "\", type - \"";
		  std::cout << connections[i]->datatype << "\"" << std::endl;
	  }
	  std::cout << std::endl;

	  const rosbag::ConnectionInfo *depthConnectInfo, *rgbConnectInfo;

	  std::cout << "Performing type check" << std::endl;
	  int itemp = 0;
	  for(int i=0; i<connections.size(); ++i )  {
	    if( connections[i]->topic == "/camera/rgb/image_color" &&
	        connections[i]->datatype == "sensor_msgs/Image" )
	    {
	      rgbConnectInfo = connections[i];
	      itemp++;
	      continue;
	    }
	    if( connections[i]->topic == "/camera/depth_registered/image_raw" &&
	        connections[i]->datatype == "sensor_msgs/Image" )
	    {
	      depthConnectInfo = connections[i];
	      itemp++;
	      continue;
	    }
	  }
	  if( itemp != 2 )  {
		  std::cout << "  Error, did not find both rgb and depth sensor_msgs/Image's in the bag";
		  std::cout << std::endl << std::endl;
	    return false;
	  }
	  else  {
		  std::cout << "  Succeeded" << std::endl << std::endl;
	  }

	  // now lets repeat with the topic names
	  rosbag::View rgbView(   *bag_handle, rosbag::TopicQuery("/camera/rgb/image_color") );
	  rosbag::View depthView( *bag_handle, rosbag::TopicQuery("/camera/depth_registered/image_raw") );

	  std::cout << "There are " << rgbView.size() << " color image(s) and ";
	  std::cout << depthView.size() << " depth image(s)" << std::endl;

	  int totalCount = (rgbView.size() + depthView.size() )/2.0;
	  double totalMovieTime = movieLength.toSec();

	  if( rgbView.size() == 0 || depthView.size() == 0 )  {
		  std::cout << "Error: zero images reported for one of the channels ... weird";
		  std::cout << std::endl << "Exiting" << std::endl << std::endl;
		  return false;
	  }
	  return true;
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

