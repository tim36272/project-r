/*
 * Utility.cpp
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
#include "Utility.h"

static const int kMaxCombineTimes = 10;
static const int NO_MEDIAN(-1);
static const int kSymmetryTolerance(0);

namespace utility {
ColorPair GetColors(const Contour& contour,const cv::Mat& image) {
	//TODO:generalize this
	//get upper half and lower half average colors

	//TODO:some kind of averaging
	//just grab it from the center
	//get color from upper half and lower half
	ColorPair colors;
	cv::Rect bound = cv::boundingRect(contour);
	colors.first = At(image,Center(UpperHalf(bound)));
	colors.second = At(image,Center(LowerHalf(bound)));
	return colors;
}
bool Overlap(const cv::Rect& first, const cv::Rect& second) {
	if(first.x < second.x) {
		if(first.y < second.y) {
			if(first.y+first.height < second.y) {
				return false;
			}
		}
		else {
			if(second.y+second.height < first.y) {
				return false;
			}
		}
		//if we made it this far check x
		if(first.x+first.width < second.x) {
			return false;
		}
	}
	else {
		if(first.y < second.y) {
			if(first.y+first.height < second.y) {
				return false;
			}
		}
		else {
			if(second.y+second.height < first.y) {
				return false;
			}
		}
		//if we made it this far check x
		if(second.x+second.width < first.x) {
			return false;
		}
	}
	return true;
}
void CombineContours(const cv::Size& mat_size,ContourList* contours) {
	if(contours->size()<=1) {
		return;
	}
	//draw them on a mat
	ContourList temp_contours;
	cv::Mat all_contours(mat_size,CV_8UC1,cv::Scalar(0));
	cv::drawContours(all_contours,*contours,-1,cv::Scalar(255),-1);

	//try dilate until there is one contour or kMaxCombineTimes
	for(int i=0;i<kMaxCombineTimes;i++) {
		//dilate the contours
		cv::dilate(all_contours,all_contours,cv::Mat());
		//see if we only have one contour
		cv::findContours(all_contours,temp_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		if(temp_contours.size()==1) {
			break;
		}
	}
	if(temp_contours.size() < contours->size()) {
		//at least we accomplished something
		temp_contours.swap(*contours);
		std::cout<<"ended up with one contour!"<<std::endl;
	}
	else {
		std::cout<<"ended up with more than one contour :("<<std::endl;
	}
}
cv::Rect UpperHalf(const cv::Rect& bound) {
	return cv::Rect(bound.x,bound.y,bound.width,bound.height/2);
}
cv::Rect LowerHalf(const cv::Rect& bound) {
	return cv::Rect(bound.x+bound.height/2,bound.y,bound.width,bound.height/2);
}
cv::Point Center(const cv::Rect& bound) {
	assert(bound.width > 0 && bound.height > 0 && bound.x >= 0 && bound.y >= 0);
	return cv::Point(bound.x+bound.width/2,bound.y+bound.height/2);
}
cv::Scalar At(const cv::Mat frame, const cv::Point location) {
	//TODO: remove Vec3b assumption
	cv::Scalar out;
	out[0] = frame.at<cv::Vec3b>(location)[0];
	out[1] = frame.at<cv::Vec3b>(location)[1];
	out[2] = frame.at<cv::Vec3b>(location)[2];

	return out;
}
void getCandidates(const cv::Rect& blob_bound, ContourList& contours, ContourList* candidates) {
	ContourListConstIt contour_cursor = contours.begin();
	ContourList list_to_replace_contours_with;
	for(;contour_cursor!=contours.end();contour_cursor++) {
		cv::Rect contour_bound = cv::boundingRect(*contour_cursor);
		if(utility::Overlap(contour_bound,blob_bound)) {
			//this contour is close so add it as a candidate
			candidates->push_back(*contour_cursor);
		}
		else {
			list_to_replace_contours_with.push_back(*contour_cursor);
		}
	}
	contours.swap(list_to_replace_contours_with);
}
ContourListIt findLargestContour(ContourList& contours) {
	int max_area=0;
	ContourListIt max_at;
	ContourListIt candidate_cursor = contours.begin();
	for(;candidate_cursor!=contours.end();candidate_cursor++) {
		int area = cv::contourArea(*candidate_cursor);
		if( area > max_area) {
			max_at = candidate_cursor;
			max_area = area;
		}
	}
	return max_at;
}
void merge(ContourList& first,const ContourList& second) {
	for(ContourListConstIt cursor=second.begin();cursor!=second.end();cursor++) {
		first.push_back(*cursor);
	}
}
void serializeContour(const Contour& contour,std::vector<int>& out) {
	//for each point in the contour, put it in the blob
	ContourConstIt cursor = contour.begin();
	for(;cursor!=contour.end();cursor++) {
		out.push_back(cursor->x);
		out.push_back(cursor->y);
	}
}

void deSerializeContour(const std::vector<int>& in,Contour& out) {
	out.clear();
	//for each point in the contour, put it in the blob
	std::vector<int>::const_iterator cursor = in.begin();
	assert(in.size()%2==0);
	for(;cursor!=in.end();) {
		cv::Point pt(*(cursor++),*(cursor++));
		int temp = pt.x;
		pt.x = pt.y;
		pt.y=temp;
		out.push_back(pt);
	}
}

void getUpwardProjection(const cv::Mat& input,std::vector<int>* output) {
	if(output==NULL)
		output = new std::vector<int>;
	output->resize(input.cols,0);
	//need to loop through entire mat
	assert(input.type()==CV_8U);
	assert(input.channels()==1);
	uchar* data = input.data;

	for(int i=0;i<input.rows*input.cols;i++) {
		if(*data !=0) {
			(*output)[i%input.cols]++;
		}
		data++;
	}
}

int getHorizontalMedian(const cv::Mat& input) {
	std::vector<int> nonblack_pixels_in_this_row;
	std::vector<int> medians;

	assert(input.type()==CV_8U);
	assert(input.channels()==1);

	//foreach pixel in input
	uchar* data = input.data;
	for(int y=0;y<input.rows;y++) {
		nonblack_pixels_in_this_row.clear();
		for(int x=0;x<input.cols;x++) {
			//if the pixel is foreground, as in not black
			if(*data !=0) {
				//add to that row's count
				nonblack_pixels_in_this_row.push_back(x);
			}
			data++;
		}
		//the median is then the center of that array
		//round down just for simplicity
		if(!nonblack_pixels_in_this_row.empty()) {
			int median_pixel_location = nonblack_pixels_in_this_row[nonblack_pixels_in_this_row.size()/2];
			medians.push_back(median_pixel_location);
		} else {
			//the row was all black, mark it as such
			medians.push_back(NO_MEDIAN);
		}
	}
	//calculate the average median
	int average_median = 0;
	int median_count=0;
	for(std::vector<int>::iterator it=medians.begin();it!=medians.end();it++) {
		if(*it != NO_MEDIAN) {
			average_median += *it;
			median_count++;
		}
	}
	if(median_count!=0) {
		return average_median/median_count;
	} else {
		return 0;
	}
}
void recolorNonSymmetricRegions(int symmetry_axis,cv::Mat* image) {
	//for each row in the image
	 uchar* data = image->data;
	 for(int i=0;i<image->rows;i++) {
		 uchar* row_begin = data;
		 //at each row, calculate left and right segment lengths
		 //corresponds to pl and pr in paper
		 int left_length,right_length;

		 int j=0;
		 //find first white pixel
		 for(;j<image->cols && *data==0;j++) {
			 data++;
		 }
		 //nothing to be done here if we got to the right of the image
		 if(j==image->cols) continue;

		 //calculate left_length
		 left_length = symmetry_axis-j;
		 //advance to axis
		 for(;j<image->cols && j < symmetry_axis;j++) {
			 data++;
		 }

		 //find last white pixel
		 for(int j=0;j<image->cols;j++) {
			 if(*data!=0) {
				 right_length = j-symmetry_axis;
			 }
			 data++;
		 }
		 int min_length = (left_length < right_length) ? left_length : right_length;
		 //classify each pixel
		 data = row_begin;
		 for(j=0;j<image->cols;j++) {
			 //for every nonzero pixel
			 if(*data!=0) {
				 //if it is not within the min length+error reclassify it
				 if(abs(j-symmetry_axis)-kSymmetryTolerance > min_length) {
					 *data = 128;
				 }
			 }
			 data++;
		 }
	 }
}

cv::Point2f centroid(const cv::Moments& moments) {
	return cv::Point2f( moments.m10/moments.m00 , moments.m01/moments.m00 );
}
}
