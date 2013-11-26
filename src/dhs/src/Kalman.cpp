/*
 * Kalman.cpp
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

#include "Kalman.h"
Kalman::Kalman(const cv::Rect& measurement) {
	//this whole function uses Mat::at for convenience, which is slow, but
	//this code shouldn't be called often

	//initialize the filters
	position_filter_.init(4,2,0);
	dimension_filter_.init(4,2,0);

	//setup the transition matrix
	cv::Mat transition(4,4,CV_32FC1);
	cv::setIdentity(transition,cv::Scalar(1));
	transition.copyTo(dimension_filter_.transitionMatrix);
	transition.at<float>(0,2) = 1;
	transition.at<float>(1,3) = 1;
		//apply it to both filters because they are 4 dimensional, 2 measurement filters
	transition.copyTo(position_filter_.transitionMatrix);


	//initialize filter's internals
		//position
		cv::setIdentity(position_filter_.measurementMatrix, cv::Scalar(1)); //1 //2 rows x 4 cols
		cv::setIdentity(position_filter_.processNoiseCov, cv::Scalar(0.001)); //.00001 //
		cv::setIdentity(position_filter_.measurementNoiseCov, cv::Scalar(0.9)); //.1 a low number here favors measurements over history //.00005
		cv::setIdentity(position_filter_.errorCovPost, cv::Scalar(.1)); //.1
//		cv::setIdentity(position_filter_.errorCovPre, cv::Scalar::all(.5)); //none
//		cv::setIdentity(position_filter_.gain, cv::Scalar::all(.9)); //none



		//dimensions
		cv::setIdentity(dimension_filter_.measurementMatrix, cv::Scalar(1)); //1
		cv::setIdentity(dimension_filter_.processNoiseCov, cv::Scalar(0.00001)); //.00001
		cv::setIdentity(dimension_filter_.measurementNoiseCov, cv::Scalar(0.001)); //.1
		cv::setIdentity(dimension_filter_.errorCovPost, cv::Scalar(.1)); //.1
//		cv::setIdentity(dimension_filter_.errorCovPre, cv::Scalar::all(.5));//none
//		cv::setIdentity(dimension_filter_.gain, cv::Scalar::all(.9)); //none

	//initialize the state matrices
		//position
		position_filter_.statePost.at<float>(0) = measurement.x+measurement.width/2;
		position_filter_.statePost.at<float>(1) = measurement.y+measurement.height/2;
		position_filter_.statePost.at<float>(2) = 0;
		position_filter_.statePost.at<float>(3) = 0;
		position_filter_.statePost.copyTo(position_filter_.statePre);

		//dimensions
		dimension_filter_.statePost.at<float>(0) = measurement.width;
		dimension_filter_.statePost.at<float>(1) = measurement.height;
		dimension_filter_.statePost.at<float>(2) = 0;
		dimension_filter_.statePost.at<float>(3) = 0;
		dimension_filter_.statePost.copyTo(dimension_filter_.statePre);

		bounding_rect_ = measurement;
}

void Kalman::update(const cv::Rect* measurement) {
	//prediction step
		//position
		location_estimate_ = position_filter_.predict();
		//dimensions
		dimension_estimate_ = dimension_filter_.predict();

	//correction step
		if(measurement!=NULL) {
			cv::Mat position_measurement_mat(2,1,CV_32FC1);
			position_measurement_mat.at<float>(0)=measurement->x+measurement->width/2;
			position_measurement_mat.at<float>(1)=measurement->y+measurement->height/2;
			position_filter_.correct(position_measurement_mat);
			//dimensions
			cv::Mat dimension_measurement_mat(2,1,CV_32FC1);
			dimension_measurement_mat.at<float>(0)=measurement->width;
			dimension_measurement_mat.at<float>(1)=measurement->height;
			dimension_filter_.correct(dimension_measurement_mat);
		}

	//re-store that information as a cv::Rect for convenience
	bounding_rect_.x = location_estimate_.at<float>(0)-dimension_estimate_.at<float>(0)/2;
	bounding_rect_.y = location_estimate_.at<float>(1)-dimension_estimate_.at<float>(1)/2;
	bounding_rect_.width = dimension_estimate_.at<float>(0);
	bounding_rect_.height = dimension_estimate_.at<float>(1);
}
