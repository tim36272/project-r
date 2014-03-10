/*
 * Periodic.h
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

#ifndef PERIODIC_H_
#define PERIODIC_H_

#include "opencv2/opencv.hpp"
#include "Kalman.h"


class Periodic {
public:
	void setup(cv::Rect bound, const cv::Point2f& centroid, const cv::Mat& template_view);
	void addFrame(const cv::Mat& frame);
	inline bool set_up() const {return set_up_;}
	inline bool get_filter_initialized() {return filter_.initialized();}
	inline std::vector<cv::Point2f> src_points() const{return src_points_;}
	inline cv::Rect getBound() {return filter_.bound();}
private:
	cv::Rect initial_bound_;
	std::vector<cv::Rect> interest_windows_;
	std::vector<cv::Rect> interesting_regions_;
	std::vector<cv::Mat> similarities_;
	std::vector<std::vector<boost::shared_ptr<cv::Mat> > > template_views_;
	std::vector<cv::Point2f> src_points_;
	bool set_up_;
	int current_frame_;
	Kalman filter_; //not sure if this is being used properly, don't think it will work for multiple regions
};

//forward decleration
class BlobDescriptorDecoratedKBMT;

namespace periodic {
typedef BlobDescriptorDecoratedKBMT BlobType;
typedef boost::shared_ptr<BlobType> BlobPtr;
void update(const cv::Mat& rgb, std::map<int,BlobPtr>& blobs_, std::vector<int>& blobs_updated_);
}
#endif /* PERIODIC_H_ */
