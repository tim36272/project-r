#ifndef TIM_SRC_UTILITIES_CPP
#define TIM_SRC_UTILITIES_CPP

#include <iostream>
#include "BlobDescriptor.h"

static const double kMinChisqrStrongRelation=5000;
static const double kMinCorrelStrongRelation=0.7;
static const double kMinIntersectStrongRelation=15;
static const double kMinBhattacharyyaStrongRelation=0.2;
static const double kMinChisqrWeakRelation=6000;
static const double kMinCorrelWeakRelation=0.5;
static const double kMinIntersectWeakRelation=4;
static const double kMinBhattacharyyaWeakRelation=0.6;

typedef std::vector<std::vector<cv::Point> > RegionList;

namespace utility {
	cv::Point calcCenter(const cv::Rect& rectangle) {
		cv::Point center(rectangle.x+rectangle.width/2,rectangle.y+rectangle.height/2);
		return center;
	}
	cv::Point2f CalcMassCenter(const cv::Moments& moment) {
		return cv::Point2f( moment.m10/moment.m00 , moment.m01/moment.m00);
	}

	bool IsMassCenterOutsideBlob(const std::vector<cv::Point>& region, const cv::Mat& all_contours, int blob_index,cv::Point* mass_center) {
		cv::Moments moment= cv::moments(region,false);
		*mass_center = CalcMassCenter(moment);
		std::cout<<"Mass center: "<<*mass_center<<" value at mass center: "<<int(all_contours.at<cv::Vec3b>(*mass_center)[0])<<" looking for: "<<blob_index<<std::endl;
		bool mass_center_is_in_blob = all_contours.at<cv::Vec3b>(*mass_center)[0]==blob_index &&
				all_contours.at<cv::Vec3b>(*mass_center - cv::Point(0,7))[0]==blob_index &&
				all_contours.at<cv::Vec3b>(*mass_center - cv::Point(7,0))[0]==blob_index &&
				all_contours.at<cv::Vec3b>(*mass_center + cv::Point(0,7))[0]==blob_index &&
				all_contours.at<cv::Vec3b>(*mass_center + cv::Point(7,0))[0]==blob_index;
		if(mass_center_is_in_blob) {
			return false;
		}
		return true;
	}
	/*
	 * Name: SplitRegion
	 * Description: divides a region into left and right (no support for top and bottom)
	 */
	void SplitRegion(const cv::Point& mass_center, const std::vector<cv::Point>& region, RegionList* new_blobs) {
		std::vector<cv::Point> left_region,right_region;
		for(int point_index = 0; point_index <region.size(); point_index++) {
			std::cout<<"in split loop"<<std::endl;
			if(region[point_index].x<mass_center.x) {
				left_region.push_back(region[point_index]);
			}
			else {
				right_region.push_back(region[point_index]);
			}
		}
		new_blobs->push_back(left_region);
		new_blobs->push_back(right_region);
		std::cout<<"Region got split"<<std::endl;
	}

	bool CheckForInheldRect(const cv::Rect& rhs, const cv::Rect& lhs) {
		cv::Point rhs_center = utility::calcCenter(rhs);
		cv::Point lhs_center = utility::calcCenter(lhs);

		bool rhs_contains_lhs = rhs.contains(lhs_center);
		bool lhs_contains_rhs = lhs.contains(rhs_center);

		return rhs_contains_lhs || lhs_contains_rhs;
	}

	bool AreClose(const cv::Rect& lhs, const cv::Rect& rhs) {
		cv::Point lhs_center(lhs.x+lhs.width/2,lhs.y+lhs.height/2);
		cv::Point rhs_center(rhs.x+rhs.width/2,rhs.y+rhs.height/2);
		int distance = pow(lhs_center.x-rhs_center.x,2)+pow(lhs_center.y-rhs_center.y,2);
		distance = sqrt(distance);
		if(distance < (lhs.width+rhs.width)) return true;
		else return false;
	}

	void ComputeHistogram(const cv::Mat& image, cv::MatND* histogram) {
		int histSize = 180;
		float hist_range[] = {1,180};
		const float* ranges = {hist_range};
		cv::calcHist(&image,1,0,cv::Mat(),*histogram,1,&histSize,&ranges,true,false);
		cv::normalize(*histogram,*histogram,0,histogram->rows,cv::NORM_MINMAX,-1,cv::Mat());
	}

	void ComputeBackProjection( const cv::Mat& color_raw, const BlobDescriptor& blob , cv::MatND* back_projection ) {
		cv::Mat color_hsv;
		cv::cvtColor(color_raw,color_hsv,CV_BGR2HSV);
		std::vector<cv::Mat> planes;
		cv::split(color_hsv,planes);
		cv::Mat mask1,mask2;
		//make a mask to suppress low saturation
		cv::threshold(planes[1],mask1,60,255,cv::THRESH_BINARY);
		//make a mask to suppress bright pixels
		cv::threshold(planes[2],mask2,170,255,cv::THRESH_BINARY_INV);
		mask1 = mask1 & mask2;
		cv::Mat hue;
		planes[0].copyTo(hue,mask1);
		cv::MatND upper_back_projection,lower_back_projection;
		float hist_range[] = {1,180};
		const float* ranges = {hist_range};
		cv::calcBackProject(&hue,1,0,blob.upper,upper_back_projection,&ranges,1);
		cv::calcBackProject(&hue,1,0,blob.lower,lower_back_projection,&ranges,1);
		*back_projection = upper_back_projection | lower_back_projection;
	}

	bool CheckForStrongRelation(double relation,int method) {
		if(method == CV_COMP_CHISQR) {
			return relation < kMinChisqrStrongRelation;
		}
		else if (method == CV_COMP_CORREL) {
			return relation > kMinCorrelStrongRelation;
		}
		else if (method== CV_COMP_INTERSECT){
			return relation > kMinIntersectStrongRelation;
		}
		else {
			return relation < kMinBhattacharyyaStrongRelation;
		}
	}

	bool CheckForWeakRelation(double relation, int method) {
		if(method == CV_COMP_CHISQR) {
			return relation < kMinChisqrWeakRelation;
		}
		else if (method == CV_COMP_CORREL) {
			return relation > kMinCorrelWeakRelation;
		}
		else if (method== CV_COMP_INTERSECT){
			return relation > kMinIntersectWeakRelation;
		}
		else {
			return relation < kMinBhattacharyyaWeakRelation;
		}
	}

	double CompareBlobs(const BlobDescriptor& first, const BlobDescriptor& second,int method) {
		double relation = 0;
		if(method == CV_COMP_CHISQR) {
			relation += compareHist(first.upper,second.upper,CV_COMP_CHISQR);
			relation += compareHist(first.lower,second.lower,CV_COMP_CHISQR);
		}
		else if (method == CV_COMP_CORREL) {
			relation += compareHist(first.upper,second.upper,CV_COMP_CORREL);
			relation += compareHist(first.lower,second.lower,CV_COMP_CORREL);
		}
		else if (method== CV_COMP_INTERSECT){
			relation += compareHist(first.upper,second.upper,CV_COMP_INTERSECT);
			relation += compareHist(first.lower,second.lower,CV_COMP_INTERSECT);
		}
		else {
			relation += compareHist(first.upper,second.upper,CV_COMP_BHATTACHARYYA);
			relation += compareHist(first.lower,second.lower,CV_COMP_BHATTACHARYYA);
		}
		relation /= 2;
		return relation;
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
