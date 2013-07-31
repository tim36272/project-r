#ifndef TIM_SRC_UTILITIES_CPP
#define TIM_SRC_UTILITIES_CPP

#include <iostream>

namespace utility {
	cv::Point calcCenter(const cv::Rect& rectangle) {
		cv::Point center(rectangle.x+rectangle.width/2,rectangle.y+rectangle.height/2);
		return center;
	}

	void PrintHistogramComparison(const cv::MatND& rhs, const cv::MatND& lhs) {
		if(rhs.size!=lhs.size) {
			std::cout<<"histograms are different sizes"<<std::endl;
			return;
		}

		uchar* rhs_data = rhs.data;
		uchar* lhs_data = lhs.data;

		//hop through each pixel and print it side by side
		for(int index=0;index<rhs.cols*rhs.rows;index++) {
			std::cout<<int(*rhs_data)<<" "<<int(*lhs_data)<<std::endl;
			rhs_data++;
			lhs_data++;
		}
	}

	int CalculateSharedArea(const cv::Rect& lhs, const cv::Rect& rhs) {
		//find the upper left corner of the inner rectangle
		cv::Point upper_left;
		cv::Point lower_right;
		if(lhs.x > rhs.x) {
			upper_left.x = lhs.x;
		}
		else {
			upper_left.x = rhs.x;
		}
		if(lhs.y > rhs.y) {
			upper_left.y = lhs.y;
		}
		else {
			upper_left.y = rhs.y;
		}

		//find the lower right corner of the inner rectangle
		if(lhs.x+lhs.width < rhs.x+rhs.width) {
			lower_right.x = lhs.x+lhs.width;
		}
		else {
			lower_right.x = rhs.x+rhs.width;
		}
		if(lhs.y+lhs.height < rhs.y+rhs.height) {
			lower_right.y = lhs.y+lhs.height;
		}
		else {
			lower_right.y = rhs.y+rhs.height;
		}

		//check that the areas intersect
		if(upper_left.x > lower_right.x || upper_left.y > lower_right.y) {
			//they don't intersect
			return 0;
		}

		//compute the area
		int width = lower_right.x - upper_left.x;
		int height = lower_right.y - upper_left.y;
		return width*height;
	}
}
#endif
