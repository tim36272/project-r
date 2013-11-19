/*
 * Kalman.h
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

#ifndef TIM_SRC_KALMAN_H
#define TIM_SRC_KALMAN_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.h"

class Kalman {
public:
	DISALLOW_COPY_AND_ASSIGN(Kalman);

	//Disallow empty construction
	Kalman();
	Kalman(const cv::Rect& measurement);
	//measurement can be NULL, means update without measurement
	void update(const cv::Rect* measurement);
	cv::Rect bound() const {return bounding_rect_;}
private:
	cv::KalmanFilter position_filter_,dimension_filter_;
	cv::Mat location_estimate_,dimension_estimate_;
	cv::Rect bounding_rect_;
};

#endif
