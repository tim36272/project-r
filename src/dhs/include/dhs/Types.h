/*
 * types.h
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

#ifndef TYPES_H_
#define TYPES_H_
#include <utility>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>
/*
 * BlobDescriptor
 */
class BlobDescriptor;
class BlobDescriptorDecoratedKM;
class BlobDescriptorDecoratedKMT;
class BlobDescriptorDecoratedKB;

typedef boost::shared_ptr<BlobDescriptor> BlobDescriptorPtr;
typedef boost::shared_ptr<BlobDescriptorDecoratedKB> BlobDescriptorDecoratedKBPtr;
typedef boost::shared_ptr<BlobDescriptorDecoratedKM> BlobDescriptorDecoratedKMPtr;
typedef boost::shared_ptr<BlobDescriptorDecoratedKMT> BlobDescriptorDecoratedKMTPtr;
/*
 * Interactions
 */
class InteractionState;
typedef std::vector<InteractionState> Interactions;

typedef std::vector<cv::Point> Contour;
typedef Contour::iterator ContourIt;
typedef Contour::const_iterator ContourConstIt;
typedef boost::shared_ptr<Contour> ContourPtr;
typedef std::vector<Contour> ContourList;
typedef ContourList::iterator ContourListIt;
typedef ContourList::const_iterator ContourListConstIt;

typedef std::pair<cv::Scalar,cv::Scalar> ColorPair;

typedef std::vector<cv::Point2f> Point2fVec;

typedef boost::shared_ptr<cv::Mat> MatPtr;


#endif /* TYPES_H_ */
