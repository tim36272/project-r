/*
 * Utility.h
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

    This file contains functions which are likely to generalizable. In other
    words, these functions perform unit operations and are expected to provide
    a very certain functionality or answer a specific question with no other
    input needed.
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include <opencv2/opencv.hpp>
#include "Types.h"

#define ASYMMETRIC_COLOR 128

namespace utility {
/*
 * OpenCV related utilities
 */
ColorPair GetColors(const Contour& contour,const cv::Mat& image);
bool Overlap(const cv::Rect& first, const cv::Rect& second);
//Attempts to combine the contours, the number of contours in the final set will
//be less than or equal to the number in the input set
void CombineContours(const cv::Size& mat_size,ContourList* contours);
cv::Rect UpperHalf(const cv::Rect& bound);
cv::Rect LowerHalf(const cv::Rect& bound);
cv::Point Center(const cv::Rect& bound);
int distance(cv::Point first, cv::Point second);
cv::Point BottomLeft(const cv::Rect& bound);
cv::Scalar At(const cv::Mat frame, const cv::Point location);
//contours is not const because upon returning, it+candidates == input contours
void getCandidates(const cv::Rect& bound, ContourList& contours, ContourList* candidates);
//checks if a rect has grown or shrank more than factor
bool changedMoreThanFactor(const cv::Rect& first, const cv::Rect& second, double factor);
//cannot be const because it returns a non-const iterator
//but this function does not modify the ContourList
ContourListIt findLargestContour(ContourList& contours);
//merges all of second into first
void merge(ContourList& first,const ContourList& second);
void getUpwardProjection(const cv::Mat& input,std::vector<int>* output);
int getHorizontalMedian(const cv::Mat& input);
void floodConcaveRegions(cv::Mat* image);
void recolorNonSymmetricRegions(int symmetry_axis,cv::Mat* image);
void removeSymmetricRegions(int symmetry_axis,cv::Mat* image);
void visualizeVector(const std::string& name,const std::vector<double>& vec);
void visualizeVector(const std::string& name,const std::vector<float>& vec);
void visualizeVector(const std::string& name,const cv::Mat& vec_as_mat);
cv::Point2f centroid(const cv::Moments& moments);
double computeSimilarity(const cv::Mat& current_view , const cv::Mat& template_view);
cv::Mat getPowerSpectrum(const cv::Mat& data);
cv::Mat getMeanPowerSpectrum(const cv::Mat& similarities,int current_frame, int num_spectrums);
boost::shared_ptr<std::vector<int> > findSignificantPeaks(const cv::Mat& data);
bool isBagSized(const BlobDescriptorPtr& blob);
void assignBagOwner(const std::map<int,BlobDescriptorDecoratedKBPtr>& blobs, BlobDescriptorDecoratedKBPtr bag);
double separation(const cv::Rect& first_bound, const cv::Rect& second_bound);
/*
 * Templated functions
 * Defintion has to be here to compile it
 */
template <typename key,typename T>
std::vector<T> mapToVectorExcludeOne(std::map<key,T> the_map, typename std::map<key,T>::iterator excluded_key) {
	if(the_map.size()==0) return std::vector<T>();

	std::vector<T> out;
	typename std::map<int,T>::const_iterator map_it = the_map.begin();
	while(map_it != the_map.end()) {
		if(map_it != excluded_key) {
			out.push_back(map_it->second);
		}
		++map_it;
	}
	return out;
}

/*
 * Numeric utilities
 */
double mean(const std::vector<float>& data);
double stdDeviation(double mean, const std::vector<float>& data);

} //namespace utility

#endif /* UTILITY_H_ */
