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
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include <opencv2/opencv.hpp>
#include "Types.h"


namespace utility {
ColorPair GetColors(const Contour& contour,const cv::Mat& image);
bool Overlap(const cv::Rect& first, const cv::Rect& second);
//Attemps to combine the contours, the number of contours in the final set will
//be less than or equal to the number in the input set
void CombineContours(const cv::Size& mat_size,ContourList* contours);
cv::Rect UpperHalf(const cv::Rect& bound);
cv::Rect LowerHalf(const cv::Rect& bound);
cv::Point Center(const cv::Rect& bound);
cv::Scalar At(const cv::Mat frame, const cv::Point location);
//contours is not const because upon returning, it+candidates == input contours
void getCandidates(const cv::Rect& bound, ContourList& contours, ContourList* candidates);
//cannot be const because it returns a non-const iterator
//but this function does not modify the ContourList
ContourListIt findLargestContour(ContourList& contours);

//merges all of second into first
void merge(ContourList& first,const ContourList& second);

void serializeContour(const Contour& contour,std::vector<int>& blob);
void deSerializeContour(const std::vector<int>& blob,Contour& contour);

void getUpwardProjection(const cv::Mat& input,std::vector<int>* output);

int getHorizontalMedian(const cv::Mat& input);

void recolorNonSymmetricRegions(int symmetry_axis,cv::Mat* image);

cv::Point2f centroid(const cv::Moments& moments);

} //namespace utility

#endif /* UTILITY_H_ */
