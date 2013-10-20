#include <ros/ros.h>
#include "MessageFetcher.h"
#include <iostream>
#include "utility.h"
#include "dhs/person.h"
#include "dhs/bag.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define kernal 11
#define kMinRegionSize 2000
#define kMaxRegionSize 180000

struct temp_descriptor {
	cv::Rect last_location;
	int first_seen;
	int last_seen;
	int depth_guess;
	int times_seen;
};

void publish(const temp_descriptor& descriptor,
			 const cv::Mat& mask,
			 const std::vector<utility::Pair_<cv::Scalar> >& published_people,
			 const std::vector<cv::Scalar>& published_bags,
			 const cv::Mat& hsv_raw,
			 ros::Publisher& person_publisher,
			 ros::Publisher& bag_publisher);
void show_regions(const RegionList& regions);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Hinter");

	MessageFetcher ros_handle;
	cv::Mat color_raw(cv::Size(680,420),CV_8UC3,cv::Scalar(0,0,0)),
			depth_raw,
			hsv_raw,
			color_foreground,
			depth_foreground;

	RegionList regions;
	std::vector<temp_descriptor> visible_blobs;
	std::vector<utility::Pair_<cv::Scalar> > published_people;
	std::vector<cv::Scalar> published_bags;


	bool run = true;
	int frame_number=0;

		//setup publisher
		ros::NodeHandle publisher_node;

		ros::Publisher person_publisher = publisher_node.advertise<dhs::person>("r/people",1000);
		ros::Publisher bag_publisher = publisher_node.advertise<dhs::bag>("r/bags",1000);

		cv::BackgroundSubtractorMOG2 color_segmentation,depth_segmentation;
		color_segmentation.set("nmixtures",3);
		depth_segmentation.set("nmixtures",16);
		depth_segmentation.set("history",300);
		depth_segmentation.set("detectShadows",0);
		color_segmentation.set("detectShadows",1);


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
		frame_number++;
		cv::cvtColor(color_raw,hsv_raw,CV_BGR2HSV);
		//process the incoming frames
		//blur
		cv::threshold(depth_raw,depth_raw,250,255,CV_THRESH_TOZERO_INV);
		cv::Mat color_blured,depth_blured;
		cv::GaussianBlur(color_raw,color_blured,cv::Size(kernal,kernal),0);
//		cv::GaussianBlur(depth_raw,depth_blured,cv::Size(kernal,kernal),0);

		//add to segmentation
		color_segmentation.operator ()(color_blured,color_foreground);
		depth_segmentation.operator ()(depth_raw,depth_foreground);



		//erode/dilate foregrounds
		int iterations=3;
		cv::dilate(color_foreground,color_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
		cv::dilate(depth_foreground,depth_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
		cv::erode(color_foreground,color_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
		cv::erode(depth_foreground,depth_foreground,cv::Mat(),cv::Point(-1,-1),iterations);


		//threshold the shadows out of the color image
		cv::threshold(color_foreground,color_foreground,254,0,CV_THRESH_TOZERO);


		cv::Mat combined_foreground;
		combined_foreground = depth_foreground | color_foreground;

		imshow("combined",combined_foreground);

		//get regions from foregrounds
		cv::findContours(combined_foreground,regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

		//get rid of small contours
		for(int region_index=regions.size()-1;region_index>=0;region_index--) {
			int area = cv::contourArea(regions[region_index]);
			if( area < kMinRegionSize) {
				regions.erase(regions.begin()+region_index);
			}
		}
		//show regions
		show_regions(regions);
/*
		//for each new region,determine if it has been seen
		for(int region_index=0;region_index<regions.size();region_index++) {
			bool updated_this_region=false;
			cv::Mat mask(depth_raw.size(),CV_8UC1,cv::Scalar(0));
			cv::drawContours(mask,regions,region_index,cv::Scalar(255),-1,8);
			cv::Rect region_bounding_rect = cv::boundingRect(regions[region_index]);
			//get the mean depth
			cv::Scalar depth_guess = cv::mean(depth_raw,mask);
			for(int visible_blobs_index=0;visible_blobs_index<visible_blobs.size() && !updated_this_region;visible_blobs_index++) {
				if((frame_number-visible_blobs[visible_blobs_index].first_seen) >15) {

					if(utility::CheckForInheldRect(region_bounding_rect,visible_blobs[visible_blobs_index].last_location)) {
						//check if the depths match
						//mask the depth image with this contour

						//if the depths differ by less than 10% then we most likely have the same blob
						if(abs(depth_guess[0]-visible_blobs[visible_blobs_index].depth_guess)<visible_blobs[visible_blobs_index].depth_guess*0.1) {
							//average the depth guesses
							visible_blobs[visible_blobs_index].depth_guess += depth_guess[0];
							visible_blobs[visible_blobs_index].depth_guess /= 2.;
							visible_blobs[visible_blobs_index].last_location = region_bounding_rect;
							visible_blobs[visible_blobs_index].last_seen = frame_number;
							visible_blobs[visible_blobs_index].times_seen++;
							//see if it is time to publish this blob
							if(visible_blobs[visible_blobs_index].times_seen>15) {
								//publish a message about it
//								publish(visible_blobs[visible_blobs_index],mask,published_people,published_bags,hsv_raw,person_publisher,bag_publisher);
								updated_this_region = true;
								std::cout<<"Updating a region"<<std::endl;
							}
						}
					}
				}
			}
			if(!updated_this_region) {
				cv::Mat temp_mat(color_raw.size(),CV_8UC3,cv::Scalar(0));
				cv::drawContours(temp_mat,regions,region_index,cv::Scalar(255,255,rand()%255),-1,8);
				imshow("this contour",temp_mat);
				cv::waitKey(1);
				//add this as a new region to visible blobs
				temp_descriptor temp;
				temp.depth_guess = depth_guess[0];
				temp.last_location = region_bounding_rect;
				temp.last_seen = frame_number;
				temp.times_seen=1;
				temp.first_seen = frame_number;
				visible_blobs.push_back(temp);
			}
		}

		//cull visible blobs list
		for(int blob_index=0;blob_index<visible_blobs.size();blob_index++) {
			if(frame_number - visible_blobs[blob_index].first_seen >5 && visible_blobs[blob_index].times_seen<=4) {
				visible_blobs.erase(visible_blobs.begin()+blob_index);
			}
		}
		std::cout<<"# visible blobs: "<<visible_blobs.size()<<std::endl;
*/
	}

	return 0;
}
void RMask(const cv::Mat& src,cv::Rect& mask, cv::Mat* result) {
	result->zeros(src.size(),CV_8UC1);
	cv::Mat temp(src.size(),CV_8UC1,cv::Scalar(0));
	cv::rectangle(temp,mask,cv::Scalar(255),-1,8);
	src.copyTo(*result,temp);
}

void publish(const temp_descriptor& descriptor,
			 const cv::Mat& mask,
			 const std::vector<utility::Pair_<cv::Scalar> >& published_people,
			 const std::vector<cv::Scalar>& published_bags,
			 const cv::Mat& hsv_raw,
			 ros::Publisher& person_publisher,
			 ros::Publisher& bag_publisher) {
	bool is_person=true;
	//arbitrate if this is a person or bag
	//TODO: that

	if(is_person) {
		//get the color distributions
		cv::Mat upper_masked,lower_masked;
		cv::Rect upper_half = utility::UpperHalf(descriptor.last_location);
		cv::Rect lower_half = utility::LowerHalf(descriptor.last_location);

		RMask(mask,upper_half,&upper_masked);
		RMask(mask,lower_half,&lower_masked);

		hsv_raw.copyTo(upper_masked,upper_masked);
		hsv_raw.copyTo(lower_masked,lower_masked);
		imshow("upper masked",upper_masked);
		imshow("lower masked",lower_masked);


	}
}

void show_regions(const RegionList& regions) {
	cv::Mat out(cv::Size(640,480),CV_8UC3,cv::Scalar(0));
	cv::drawContours(out,regions,-1,cv::Scalar(255),-1,8);
	imshow("Contours",out);
}
