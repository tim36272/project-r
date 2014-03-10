/*
 * Periodic.cpp
    Copyright (C) 2014 Timothy Sweet

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
#include "dhs/Periodic.h"
#include <ros/ros.h>
#include "dhs/Utility.h"
#include "dhs/BlobDescriptorDecorated.h"

//static const int kMinContourArea(100);
static const int kMinObjectSize(100);

void Periodic::setup(cv::Rect bound, const cv::Point2f& centroid, const cv::Mat& template_view) {
	if(bound.x < 0) bound.x = 0;
	if(bound.y < 0) bound.y = 0;
	if(bound.width < 0) bound.width = 0;
	if(bound.height < 0) bound.height = 0;
	if(bound.x+bound.width > template_view.cols) bound.width = template_view.cols-bound.x-1;
	if(bound.y+bound.height > template_view.rows) bound.height = template_view.rows-bound.y-1;

        //store the blob's current location
        {
                initial_bound_ = bound;
                interest_windows_.clear();
                cv::Rect backpack_region(0,
                                         0,
                                         initial_bound_.width,
                                         initial_bound_.height*0.5);
                cv::Rect briefcase_region(0,
                                          initial_bound_.height*0.5,
                                          initial_bound_.width,
                                          initial_bound_.height*0.5);
                interest_windows_.push_back(backpack_region);
                interest_windows_.push_back(briefcase_region);
        }
        //push back empty space for similarity measures
        {
                similarities_.clear();
                for(int i=0;i<interest_windows_.size();i++) {
                        similarities_.push_back(cv::Mat(255,255,CV_32FC1,cv::Scalar(0)));
                }
        }
        cv::Mat cropped(template_view,initial_bound_);

        cv::Size old_size = cropped.size();
        cv::resize(cropped,cropped,cv::Size(9,15));
        cv::resize(cropped,cropped,old_size);
        //store the blob's template
        {
                template_views_.clear();
                std::vector<cv::Rect>::iterator it = interest_windows_.begin();
                std::vector<MatPtr> temp_list;
                while(it!=interest_windows_.end()) {
                    MatPtr temp_mat(new cv::Mat(cropped,*it));
                    temp_list.push_back(temp_mat);
                    it++;
                }
                template_views_.push_back(temp_list);
        }

        //store the points used for affine warp: center, upper left, lower left
        src_points_.clear();
        src_points_.push_back(centroid);
        src_points_.push_back(bound.tl());
        src_points_.push_back(utility::BottomLeft(bound));

        interesting_regions_.clear();

        set_up_=true;
        current_frame_ = 1;
}

void Periodic::addFrame(const cv::Mat& frame) {
        //get just the blob in the image
        cv::Mat blob_cropped(frame,cv::Rect(initial_bound_));
        cv::Mat original_frame(blob_cropped.clone());

        //scaling helps with tracking errors
        cv::Size old_size = blob_cropped.size();
        cv::resize(blob_cropped,blob_cropped,cv::Size(9,15),0,0,CV_INTER_CUBIC);
        cv::resize(blob_cropped,blob_cropped,old_size,0,0,CV_INTER_CUBIC);
        //store the frame
        {
                std::vector<cv::Rect>::const_iterator it = interest_windows_.begin();
                std::vector<MatPtr> temp_list;
                while(it!=interest_windows_.end()) {
                        MatPtr temp_mat(new cv::Mat(blob_cropped,*it));
                        temp_list.push_back(temp_mat);
                        it++;
                }
                template_views_.push_back(temp_list);
        }

        //loop through each interest window and determine if it is exhibiting aperiodic motion
        for(int window_index=0;window_index<interest_windows_.size();window_index++) {
                cv::Mat this_window_high_res(original_frame,interest_windows_[window_index]);
                //compute the similarity between the current frame and every other frame
                for(int template_index=0;template_index<template_views_.size()-1;template_index++) {
                        double similarity_measurement = utility::computeSimilarity(*template_views_.rbegin()->operator [](window_index),
                                                                                   *template_views_[template_index][window_index]);
                        similarities_[window_index].at<float>(template_index,current_frame_-1) = (similarity_measurement);
                        similarities_[window_index].at<float>(current_frame_-1,template_index) = (similarity_measurement);

                }
                cv::Mat power_spectrum = utility::getMeanPowerSpectrum(similarities_[window_index],current_frame_,1);

                //determine if the FT indicates periodic motion
                //by getting significant peaks in the power spectrum
                boost::shared_ptr<std::vector<int> > significant_peaks = utility::findSignificantPeaks(power_spectrum);
                if(significant_peaks->size()>0) {
                        ROS_INFO_STREAM_NAMED("BlobDescriptorExtended","there are "<<significant_peaks->size()<<" significant peaks in window "<<window_index);
                }
                if(significant_peaks->size() == 0 && current_frame_ > 40) {
                        //the motion is aperiodic, so check if there is a large blob in the mask
                        ContourList temp_contours;
                        cv::findContours(this_window_high_res,temp_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
                        int max_contour_size=-1;
                        int max_contour_index;
                        for(int contour_index=0;contour_index<temp_contours.size();contour_index++) {
                                int this_contour_size = cv::contourArea(temp_contours[contour_index]);
                                if(this_contour_size > max_contour_size) {
                                        max_contour_index = contour_index;
                                        max_contour_size = this_contour_size;
                                }
                        }
                        if(max_contour_size > kMinObjectSize) {
                                //there is a large, asymmetric thing here, so it is probably a bag and is interesting
                                cv::Rect bound = cv::boundingRect(temp_contours[max_contour_index]);
                                if(interesting_regions_.size()==0) {
                                        filter_.update(&bound);
                                        std::cout<<"object size: "<<max_contour_size<<std::endl;
                                } else {
                                        filter_.update(&bound);
                                        std::cout<<"object size: "<<max_contour_size<<std::endl;
                                }
                                ROS_DEBUG_STREAM("this frame has an interesting object, bound is: "<<bound);
                        }
                        else {
                                ROS_DEBUG_STREAM("small object size: "<<max_contour_size);
                        }

                }


                //title used to visualize similarities and FT
                std::stringstream title; title<<"Ftr:"<<window_index;
                //show similarity graph and power spectrum
                utility::visualizeVector(title.str(),similarities_[window_index].row(0).colRange(0,current_frame_));
                title<<" power";
                utility::visualizeVector(title.str(),power_spectrum);

//                title <<" sim";
//                cv::Mat temp;
//                cv::normalize(similarities_[window_index],temp,0.0,1.0,CV_MINMAX);
//                imshow(title.str(),temp);
        }
        current_frame_++;
}

namespace periodic {
void update(const cv::Mat& rgb, std::map<int,BlobPtr>& blobs_, std::vector<int>& blobs_updated_) {
	//for each blob which was updated this frame
	if(blobs_updated_.size()==0) {
		return;
	}
	cv::Mat rgb_out = rgb.clone();
	//TODO: get the size another way
	cv::Mat output(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
	std::vector<int>::iterator blob_iterator = blobs_updated_.begin();
	for(;blob_iterator!=blobs_updated_.end();) {
		//just get this blob as a pointer
		BlobPtr current_blob = blobs_.at(*blob_iterator);
		/*
		 * erase this index from the updated list and reset the iterator. This prevents
		 * jumping past the end of the vector when incrementing the iterator
		 */
		blobs_updated_.erase(blob_iterator);

		//TODO: fix the race condition here in which an empty list which
		//has a blob added to it after here will break the loop
		blob_iterator = blobs_updated_.begin();

		//draw the contour on a mat
		//TODO:get the size another way
		cv::Mat blob_visual(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
		{
			ContourList temp_list;
			temp_list.push_back(current_blob->getLastContour());
			cv::drawContours(blob_visual,temp_list,0,cv::Scalar(255),-1);
		}

		//compute the body axis
		//try computing it as the horizontal projection(as in sum of vertical stacks of pixels)
		utility::floodConcaveRegions(&blob_visual);
		int symmetry_axis;
		{
			std::vector<int> upward_projection;
			utility::getUpwardProjection(blob_visual,&upward_projection);
			int max=0;
			int max_projection_at=0;
			for(int i=0;i<upward_projection.size();i++) {
				if(upward_projection[i] > max) {
					max = upward_projection[i];
					max_projection_at=i;
				}
			}
			//draw max horizontal projection axis
			//cv::line(blob_visual,cv::Point(max_at,0),cv::Point(max_at,480),cv::Scalar(128+64),2);
			symmetry_axis = max_projection_at;
		}


		cv::Mat full_blob(blob_visual.clone());

		//classify each pixel as symmetric or asymmetric
		utility::removeSymmetricRegions(symmetry_axis,&blob_visual);
		utility::recolorNonSymmetricRegions(symmetry_axis,&full_blob); //this is only for the user's visualization

		//show the symmetry line just for the user's entertainment
		cv::line(full_blob,cv::Point(symmetry_axis,0),cv::Point(symmetry_axis,480),cv::Scalar(200),2);

		//show which regions are symmetric/asymmetric
		cv::imshow("symetric/aysmetric",full_blob);

		//check periodicity and reclassify pixels if necessary
		//algorithm: if there are no similarity measurements for this blob, it must be the first time it's been seen
		if(!current_blob->tracker_.set_up()) {
			//this is the first time the blob has been seen
			cv::Point2f centroid( current_blob->moments_.m10/current_blob->moments_.m00 , current_blob->moments_.m01/current_blob->moments_.m00 );
			current_blob->tracker_.setup(current_blob->getLastFilteredBound(),centroid,blob_visual);
		} else if(utility::changedMoreThanFactor( //the bound has changed a lot since first view, so reset all the periodicity tracking stuff
				current_blob->getRawBound(0),
				current_blob->getLastRawBound(), 1)) {
			//the blob changed a lot so dump everything and start over
			//this typcially occurs a few times as the blob is entering the scene
			ROS_DEBUG_STREAM("Resetting the tracker for id: "<<current_blob->Id()<<", centroid is: "<<current_blob->getCentroid()<<"Last filtered bound is: "<<current_blob->getLastFilteredBound());
			current_blob->tracker_.setup(current_blob->getLastFilteredBound(),current_blob->getCentroid(),blob_visual);
			current_blob->eraseHistory();
		} else {
			//blob has already been seen, just check for periodicity
			Point2fVec dst_points;
			dst_points.push_back(current_blob->getCentroid());
			dst_points.push_back(current_blob->getLastRawBound().tl());
			dst_points.push_back(utility::BottomLeft(current_blob->getLastRawBound()));

			//get an affine transformation matrix which maps the current view to the template view
			cv::Mat warp_mat = cv::getAffineTransform(dst_points,current_blob->tracker_.src_points());

			//warp the visual representation of the non symmetric parts of the blob to the reference frame (ideally the first full view of the blob)
			cv::Mat warped_blob;
			cv::warpAffine(blob_visual,warped_blob,warp_mat,blob_visual.size());

			current_blob->tracker_.addFrame(warped_blob);
		}

		//draw interesting regions
		if(current_blob->tracker_.get_filter_initialized()) {
			cv::Rect interest_rect(current_blob->getLastRawBound().x+current_blob->tracker_.getBound().x,
					current_blob->getLastRawBound().y+current_blob->tracker_.getBound().y,
					current_blob->tracker_.getBound().width,
					current_blob->tracker_.getBound().height);
			cv::rectangle(rgb_out,interest_rect,cv::Scalar(255),1);
		}
	}
	imshow("interesting regions",rgb_out);
	cv::waitKey(1);
}
}
