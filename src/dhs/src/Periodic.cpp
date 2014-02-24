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
#include "Periodic.h"

#include <ros/ros.h>

#include "Utility.h"

//static const int kMinContourArea(100);
static const int kMinObjectSize(100);

void Periodic::setup(const cv::Rect& bound, const cv::Point2f& centroid, const cv::Mat& template_view) {
        //store the blob's current location
        {
                initial_bound_ = bound;
                interest_windows_.clear();
                cv::Rect backpack_region(0,
                                                                 initial_bound_.height*0.1,
                                                                 initial_bound_.width,
                                                                 initial_bound_.height*0.4);
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
                                std::cout<<"adding an interesting object"<<std::endl;
                        }
                        else {
                                std::cout<<"small object size: "<<max_contour_size<<std::endl;
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
