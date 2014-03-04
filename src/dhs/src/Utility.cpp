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
#include "dhs/Utility.h"

#include <cmath>
#include <string>
#include <sstream>
#include <numeric>

#include <ros/ros.h>

#include "dhs/BlobDescriptor.h"

static const int kMaxCombineTimes(10);
static const int NO_MEDIAN(-1);
static const int kSymmetryTolerance(5); //TODO: make this set dynamically


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
		ROS_DEBUG_STREAM_NAMED("utility","ended up with one contour!");
	}
	else {
		ROS_DEBUG_STREAM_NAMED("utility","ended up with more than one contour :(");
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
int distance(cv::Point first, cv::Point second) {
	return sqrt( (first.x-second.x)*(first.x-second.x) + (first.y-second.y)*(first.y-second.y) );
}
cv::Point BottomLeft(const cv::Rect& bound) {
	assert(bound.width > 0 && bound.height > 0);
	return cv::Point(bound.x,bound.y+bound.height);
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
bool changedMoreThanFactor(const cv::Rect& first, const cv::Rect& second, double factor) {
	double first_aspect_ratio = ((double)first.width)/first.height;
	double second_aspect_ratio = ((double)second.width)/second.height;
	//ROS_DEBUG_STREAM("first aspect ratio: "<<first_aspect_ratio<<" second: "<<second_aspect_ratio);
	//ROS_DEBUG_STREAM("first check: "<<std::abs(first_aspect_ratio-second_aspect_ratio)/first_aspect_ratio<<" > "<<factor);
	//ROS_DEBUG_STREAM("second check: "<<std::abs(first_aspect_ratio-second_aspect_ratio)/second_aspect_ratio<<" > "<<factor);

	//check if aspect ratio changed dramatically
	if(std::abs(first_aspect_ratio-second_aspect_ratio)/first_aspect_ratio > factor) return true;
	if(std::abs(first_aspect_ratio-second_aspect_ratio)/second_aspect_ratio > factor) return true;

	//check if height or width changed dramatically
	if(abs(first.height-second.height) > first.height) return true;
	if(abs(first.height-second.height) > second.height) return true;

	if(abs(first.width-second.width) > first.width) return true;
	if(abs(first.width-second.width) > second.width) return true;


	return false;

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
void floodConcaveRegions(cv::Mat* image) {
	uchar* data = image->data;
	for(int y=0;y<image->rows;y++) {
		//find a nonblack pixel
		int x=0;
		for(;x<image->cols;x++) {
			if(*data!=0) break;
			data++;
		}
		if(x==image->cols) continue; //no nonblack pixels in this row
		//find a black pixel
		for(;x<image->cols;x++) {
			if(*data==0) break;
			data++;
		}
		if(x==image->cols) continue; //only one block of nonpixels in this row
		while(x<image->cols) {
			data++;
			x++;
			if(*data==0) continue;

			//so there's another white pixel, fill in the gap
			//guaranteed to find a white pixel because it was found earlier
			//not multithread safe
			uchar* data_reverse = data-1;
			while(*data_reverse==0) {
				*data_reverse = 255;
				data_reverse--;
			}
		}
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
		 //find first nonblack pixel
		 for(;j < image->cols && *data==0;j++) {
			 data++;
		 }
		 //nothing to be done here if we got to the right of the image
		 //AKA there were no nonzero pixels in this row
		 if(j==image->cols) continue;

		 //calculate left_length
		 left_length = symmetry_axis-j;
		 //advance to axis
		 for(;j < image->cols && j < symmetry_axis;j++) {
			 data++;
		 }

		 //find last white pixel
		 for(;j<image->cols;j++) {
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
					 *data = ASYMMETRIC_COLOR;
				 }
			 }
			 data++;
		 }
	 }
}

void removeSymmetricRegions(int symmetry_axis,cv::Mat* image) {
	//get the recolored image
	recolorNonSymmetricRegions(symmetry_axis,image);
	//remove high values
	cv::threshold(*image,*image,ASYMMETRIC_COLOR+1,0,cv::THRESH_TOZERO_INV);
}

cv::Point2f centroid(const cv::Moments& moments) {
	return cv::Point2f( moments.m10/moments.m00 , moments.m01/moments.m00 );
}

void visualizeVector(const std::string& name,const std::vector<double>& vec) {
	//find the max
	double max = *std::max_element(vec.begin()+1,vec.end());
	double min = *std::min_element(vec.begin()+1,vec.end());
	//make a mat to display it
	cv::Mat graph(100,vec.size(),CV_8UC1,cv::Scalar::all(0));
	//plot the points
	std::vector<double>::const_iterator it = vec.begin()+1;
	int i=1;
	while(it!=vec.end()) {
		cv::circle(graph,cv::Point(i,100-(*it-min)/(max-min)*100.),1,cv::Scalar(255),-1);
		it++; i++;
	}
	//write the max on the mat
	std::stringstream max_text;
	max_text<<max;
	cv::putText(graph,max_text.str(),cv::Point(0,10),CV_FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(128),1);
	cv::imshow(name,graph);
}
void visualizeVector(const std::string& name,const std::vector<float>& vec) {
	//find the max
	double max = *std::max_element(vec.begin()+1,vec.end());
	double min = *std::min_element(vec.begin()+1,vec.end());
	if(max==min) min = max-1;
	//make a mat to display it
	cv::Mat graph(100,vec.size(),CV_8UC1,cv::Scalar::all(0));
	//plot the points
	std::vector<float>::const_iterator it = vec.begin()+1;
	int i=1;
	while(it!=vec.end()) {
		cv::circle(graph,cv::Point(i,100-(*it-min)/(max-min)*100.),1,cv::Scalar(255),-1);
		it++; i++;
	}
	//write the max on the mat
	std::stringstream max_text;
	max_text<<max;
	cv::putText(graph,max_text.str(),cv::Point(0,10),CV_FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(128),1);
	cv::imshow(name,graph);
}
void visualizeVector(const std::string& name,const cv::Mat& vec_as_mat) {
	assert(vec_as_mat.rows==1);
	assert(vec_as_mat.type()==CV_32FC1);
	std::vector<float> vec;
	for(int i=0;i<vec_as_mat.cols;++i) {
		vec.push_back(vec_as_mat.at<float>(i));
	}
	//find the max
	double max = *std::max_element(vec.begin()+1,vec.end());
	double min = *std::min_element(vec.begin()+1,vec.end());
	if(max==min) min = max-1;
	//make a mat to display it
	cv::Mat graph(100,vec.size(),CV_8UC1,cv::Scalar::all(0));
	//plot the points
	std::vector<float>::const_iterator it = vec.begin()+1;
	int i=1;
	while(it!=vec.end()) {
		cv::circle(graph,cv::Point(i,100-(*it-min)/(max-min)*100.),1,cv::Scalar(255),-1);
		it++; i++;
	}
	//write the max on the mat
	std::stringstream max_text;
	max_text<<max;
	cv::putText(graph,max_text.str(),cv::Point(0,10),CV_FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(128),1);
	cv::imshow(name,graph);
}
double mean(const std::vector<float>& data) {
	return std::accumulate(data.begin(),data.end(),0)/(double)data.size();
}
double stdDeviation(double mean,const std::vector<float>& data) {
	return std::sqrt(std::inner_product(data.begin(), data.end(), data.begin(), 0.0)/data.size() - mean * mean);
}
double computeSimilarity(const cv::Mat& current_view , const cv::Mat& template_view) {
	//it is just looking in the window where a backpack is expected to be

	cv::Mat difference_from_template = current_view - template_view |
									   template_view - current_view;
	imshow("current view",current_view);
	imshow("template view",template_view);

	//compute the similarity between this frame and the template
	//or, more specifically, how many differences there are
	return 1./((double)cv::sum(difference_from_template)[0]+1);
}
cv::Mat getPowerSpectrum(const cv::Mat& data) {
	assert(data.rows==1);
	cv::Mat power_spectrum(1,data.cols,CV_32FC1,cv::Scalar(0));

	//this just carries the similarity data for the FT
	cv::Mat complex_data(1,data.cols,CV_32FC2,cv::Scalar::all(0));
	assert(data.depth()==complex_data.depth());

	//copy similarities data into the complex mat
	for(int i=0;i<data.cols;i++) {
		complex_data.at<cv::Vec2f>(i)[0] = data.at<float>(i);
	}



	dft(complex_data,complex_data);

	//center the spectra
	//TODO: this doesn't seem to work...
/*	if(complex_data.cols>1) {
		int center = complex_data.cols;
		cv::Mat left(complex_data,cv::Rect(0,0,center/2,1));
		cv::Mat right(complex_data,cv::Rect(center/2,0,center/2-1,1));
		cv::Mat temp(right.clone());
		left.copyTo(right);
		temp.copyTo(left);
	}
*/
	//calculate power spectrum of the FT

	//skips the first data point because that one tends to skew resultsa
	power_spectrum.at<float>(0) = 0;
	for(int power_index=1;power_index<complex_data.cols;++power_index) {
		power_spectrum.at<float>(power_index) = pow(complex_data.at<cv::Vec2f>(power_index)[0],2) +
												pow(complex_data.at<cv::Vec2f>(power_index)[1],2);
	}
	return power_spectrum;
}
cv::Mat getMeanPowerSpectrum(const cv::Mat& similarities,int current_frame, int num_spectrums) {
	//get the power spectrum of each similarity graph
	int starting_point = current_frame-num_spectrums;
	if(starting_point < 0) starting_point = 0;

	cv::Mat power_spectrums(num_spectrums,current_frame,CV_32FC1);
	int power_spectrum_index=0;
	for(int i=starting_point;i<current_frame;++i) {
		cv::Mat temp = getPowerSpectrum(similarities.row(i).colRange(0,current_frame));
		for(int j=0;j<temp.cols;j++) {
			power_spectrums.row(power_spectrum_index).colRange(0,current_frame).at<float>(j) = temp.at<float>(j);
		}
		power_spectrum_index++;
	}

	cv::Mat mean_spectrum(1,current_frame,CV_32FC1);
	for(int similarity_index=0;similarity_index<current_frame;++similarity_index) {
		mean_spectrum.at<float>(similarity_index) = cv::mean(power_spectrums.col(similarity_index))[0];
	}
	return mean_spectrum;
}
boost::shared_ptr<std::vector<int> > findSignificantPeaks(const cv::Mat& data) {
	assert(data.rows==1);
	boost::shared_ptr<std::vector<int> > significant_peaks(new std::vector<int>);
	//find the mean and standard deviation of the power spectrum
	cv::Scalar mean, stdev;
	cv::meanStdDev(data,mean,stdev);
	double threshold = mean.val[0] + 6 * stdev.val[0];


	int frequency_index=0;
	float max=0;
	for(int i=0;i<data.cols;i++) {
		if(data.at<float>(i) >  max) {
			max = data.at<float>(i);
		}
		if(data.at<float>(i) > threshold) {
			significant_peaks->push_back(frequency_index);
		}
		frequency_index++;
	}
	return significant_peaks;
}
bool isBagSized(const BlobDescriptorPtr& blob) {
	//(temporary) hack: if the blob satisfies
	//(height)/(200-depth) < 3.0
	//it is a bag
	return blob->getLastRawBound().height/(200-blob->getLastDepth()) < 3;
}

} // namespace utility
