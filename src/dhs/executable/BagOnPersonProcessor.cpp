/*
 * BagOnPersonProcessor.cpp
    Copyright (C) 2013  Timothy Sweet

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//std includes
#include <sstream>

//library includes
#include <ros/ros.h>

//project includes
#include "../include/Utility.h"
#include "../include/ProcessorNode.h"
#include "../include/EventCodes.h"
#include "../include/BlobDescriptor.h"

class BagOnPersonProcessor : public ProcessorNode {
public:
	BagOnPersonProcessor(const std::string& blob_topic,const std::string& event_topic) :
		input_stream_(blob_topic) {
		event_name_ = BagOnPersonName;
		event_code_ = BagOnPersonCode;
		output_topic_ = event_topic;
		init();
		std::cout<<"subscribed to: "<<input_stream_.getTopic()<<std::endl;
	}
	void callback(const ros::TimerEvent& event);

	void init();
private:
	DISALLOW_COPY_AND_ASSIGN(BagOnPersonProcessor);
	DISALLOW_DEFAULT_CONSTRUCTION(BagOnPersonProcessor);
	BlobDescriptorFetcher input_stream_;
	//TODO: combine these into a single data structure
	//a relationship between blob IDs and their past similarities
	std::map<int,std::vector<double> > similarities_;
	//a relationship between blob IDs and their first frame contour
	std::map<int,cv::Mat> templates_;
	//a relationship between blob IDs and their centroids
	std::map<int,cv::Point2f> centroids_;
};

int main(int argc, char* argv[]) {
	//connect to ros
	ros::init(argc, argv, "segmentation");
	ros::NodeHandle handle("~");

	BagOnPersonProcessor processor("blobs_in","events_out");
	//loop at 60 hz just because that seems like a pretty good rate given
	//that the camera is running at 30 hz
	ros::Timer timer = handle.createTimer(ros::Duration(1./30.), &BagOnPersonProcessor::callback, &processor);

	ros::spin();

	return 0;
}

void BagOnPersonProcessor::init() {
	setupOutput();
}

void BagOnPersonProcessor::callback(const ros::TimerEvent& event) {
	//for each blob which was updated this frame
	if(input_stream_.blobs_updated_.size()==0) {
		return;
	}
	//TODO: get the size another way
	cv::Mat output(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
	std::vector<int>::iterator blob_iterator = input_stream_.blobs_updated_.begin();
	for(;blob_iterator!=input_stream_.blobs_updated_.end();) {
		//just store the index as an int to make things easier
		int cursor = *blob_iterator;
		//erase it from the list and reset the iterator. This prevents
		//jumping past the end of the vector when incrementing the iterator
		input_stream_.blobs_updated_.erase(blob_iterator);
		//TODO: fix the race condition here in which an empty list which
		//has a blob addd to it after here will break the loop
		blob_iterator = input_stream_.blobs_updated_.begin();

		//update blob number cursor, look it up in the map

		//draw the contour on a mat
		//TODO:get the size another way
		cv::Mat blob_visual(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
		ContourList temp_list;
		temp_list.push_back(input_stream_.data_[cursor]->contour_);
		cv::drawContours(blob_visual,temp_list,0,cv::Scalar(255),-1);
		imshow("redrawn contour",blob_visual);


		//compute the body axis
		//option 1: try computing it as the center of mass
		 /// Get the moment
		  cv::Moments mu = cv::moments( input_stream_.data_[cursor]->contour_, false );

		  ///  Get the mass center
		  cv::Point2f mc( mu.m10/mu.m00 , mu.m01/mu.m00 );
		//option 2: try computing it as the horizontal projection(as in sum of vertical stacks of pixels)
		  std::vector<int> upward_projection;
		utility::getUpwardProjection(blob_visual,&upward_projection);
		std::cout<<"upward projection of "<<cursor<<std::endl;
		int max=0;
		int max_at=0;
		for(int i=0;i<upward_projection.size();i++) {
			if(upward_projection[i] > max) {
				max = upward_projection[i];
				max_at=i;
			}
			std::cout<<upward_projection[i]<<" ";
		}
		std::cout<<std::endl;
		//option 3: calculate the median line
		int median_at = utility::getHorizontalMedian(blob_visual);
		//draw the mass center axis
		//cv::line(blob_visual,cv::Point(mc.x,0),cv::Point(mc.x,480),cv::Scalar(128),2);
		//draw max upward projection axis
		cv::line(blob_visual,cv::Point(max_at,0),cv::Point(max_at,480),cv::Scalar(128+64),2);
		//draw median line
		//cv::line(blob_visual,cv::Point(median_at,0),cv::Point(median_at,480),cv::Scalar(128),2);

		int symmetry_axis = max_at;

		//classify each pixel as symmetric or asymmetric
		utility::recolorNonSymmetricRegions(symmetry_axis,&blob_visual);

		//check periodicity and reclassify pixels if necessary
		//if this is the first time the blob is being seen, set up its template
		//store the blob_visual for later comparison
		//store maximum similarity in the similarity tracker
		//TODO: make inherited class of BlobDescriptor which includes this stuff

		//get an image of just the blob
//		cv::Mat cropped_blob;
		//should I use CurrentBound (aka the filtered version) instead?
//		cv::Mat(blob_visual,input_stream_.data_[cursor]->LastRawBound()).copyTo(cropped_blob);
		if(input_stream_.data_[cursor]->FirstSeen()==input_stream_.data_[cursor]->LastSeen()) {

			//this is the first time the blob is seen: set up its template
			//this is guaranteed to make a new element because this is a new blob
			templates_.insert(std::pair<int,cv::Mat>(cursor,blob_visual));

			centroids_.insert(std::pair<int,cv::Point2f>(cursor,input_stream_.data_[cursor]->getCentroid() ));

			//store a maximum similarity of 1
			similarities_.insert(std::pair<int,std::vector<double> >( cursor , std::vector<double>(1,1.0) ));
		} else {
			//the blob already exists, so update it by computing the similarity
			//scale and align the blob using affine transformation
			//we already have centers, so just need two more points. The corners would be good
			cv::Point2f src_points[3];
			cv::Point2f dst_points[3];

			src_points[0] = centroids_.at(cursor);
			dst_points[0] = input_stream_.data_[cursor]->getCentroid();

			src_points[1] = input_stream_.data_[cursor]->getBound(0).tl();
			dst_points[1] = input_stream_.data_[cursor]->LastRawBound().tl();

			src_points[1] = input_stream_.data_[cursor]->getBound(0).br();
			dst_points[1] = input_stream_.data_[cursor]->LastRawBound().br();

			cv::Mat warp_mat = cv::getAffineTransform(src_points,dst_points);

			cv::Mat warped_blob;
			cv::warpAffine(blob_visual,warped_blob,warp_mat,blob_visual.size());
			imshow("current blob before",blob_visual);
			imshow("first blob",templates_.at(cursor));
			imshow("current blob warped",warped_blob);



		}
		//store results

		//show results - a mat with regions classified appropriately
		output = output | blob_visual;
	}
	imshow("axis",output);
	cv::waitKey(1);
}
