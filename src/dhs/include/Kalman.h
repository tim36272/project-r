#ifndef TIM_SRC_KALMAN_H
#define TIM_SRC_KALMAN_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class Kalman {
public:
	Kalman() {

	}
	Kalman(const Kalman& rhs) {
		assert(false);
	}
	Kalman& operator=(const Kalman& rhs) {
		position_filter_ = rhs.position_filter_;
		dimension_filter_ = rhs.dimension_filter_;
		rhs.location_estimate_.copyTo(location_estimate_);
		rhs.dimension_estimate_.copyTo(dimension_estimate_);
		bounding_rect_ = rhs.bounding_rect_;
		return *this;
	}
	void init(const cv::Rect& measurement);
	void update(const cv::Rect* measurement);
	cv::Rect bounding_rect() const {return bounding_rect_;}
	cv::Point bounding_rect_origin() const {return bounding_rect_.tl();}
private:
	cv::KalmanFilter position_filter_,dimension_filter_;
	cv::Mat location_estimate_,dimension_estimate_;
	cv::Rect bounding_rect_;
};

#endif
