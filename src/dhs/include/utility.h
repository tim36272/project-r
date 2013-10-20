/*
 * utility.h
 *
 *  Created on: Sep 4, 2013
 *      Author: tim
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <ros/console.h>
#include <iostream>
#include "BlobDescriptor.h"
#include "Types.h"

	typedef std::vector<cv::Point> Region;
	typedef std::vector<Region> RegionList;
	typedef std::vector<PersonDescriptor> PersonList;
	typedef std::vector<BagDescriptor> BagList;
	typedef const cv::Mat& kMat;

namespace utility {

	cv::Point calcCenter(const cv::Rect& rectangle);
	void SplitRegion(const cv::Point& mass_center, cv::Mat& region, RegionList* new_blobs);
	void CombineRegions(cv::Mat& region, RegionList* new_blobs);
	void ComputeHistogram(const cv::Mat& image, const cv::Mat& mask, cv::MatND* histogram);
	cv::Mat MakeBinaryMask(const cv::Mat& image);
	bool AreVerticallyAligned(const Region& first, const Region& second);
	bool AreVerticallyAlignedOriented(const Region& upper, const Region& lower);
	bool IsWide(const Region& region);
	bool RectsOverlap(const cv::Rect& rhs, const cv::Rect& lhs);
	void GetCandidates(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates);
	void GetCandidatesFull(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates);
	int CheckForPersonUsingColorScheme(const PersonList& known_blobs,cv::Scalar upper_color,cv::Scalar lower_color);
	int CheckForBagUsingColorScheme(const BagList& known_bags,cv::Scalar color);
	void DeleteRegion(int region_index,RegionList* regions);
	void GetSignificantRegions(const cv::Size& frame_size, const std::vector<cv::Rect>& candidate_areas,const cv::Mat& projection,RegionList* regions);
	cv::Rect CalcOrientedBoundingRect(const cv::Rect& upper, const cv::Rect& lower);
	void NewPerson(const cv::Mat& depth_raw, const cv::Rect& upper, const cv::Rect& lower,const utility::Pair_<cv::Scalar> color, int frame_number, PersonDescriptor* person);
	void NewBag(const cv::Mat& depth_raw, const cv::Rect& region, const cv::Scalar& color, int frame_number, BagDescriptor* bag);
	void GetSpecificColorMap(const cv::Mat& hsv_raw,const cv::Scalar& color,cv::Mat* out);
	void MaskWithDepth(const cv::Mat& depth_raw, int target_depth,const cv::Mat& in,cv::Mat* out);
	cv::Rect UpperHalf(const cv::Rect in);
	cv::Rect LowerHalf(const cv::Rect in);
	cv::Rect UpperPersonHalf(const cv::Rect in);
	cv::Rect LowerPersonHalf(const cv::Rect in);
	void CullList(int frame_number, PersonList* known_people);
	void CullList(int frame_number, BagList* known_bags);
	void UpdatePerson(const cv::Mat& depth_raw, const cv::Scalar& upper_observed_color,const cv::Scalar& lower_observed_color,int frame_number, const cv::Rect& upper,const cv::Rect& lower, PersonDescriptor* person);
	void UpdateBag(const cv::Mat& depth_raw, const cv::Scalar& observed_color,int frame_number, const cv::Rect& region, BagDescriptor* bag);
	utility::Pair TryToUpdatePerson(const cv::Mat& depth_raw, int frame_number, const RegionList& upper_regions, const RegionList& lower_regions, PersonDescriptor* person);
	int TryToUpdateBag(const cv::Mat& depth_raw, int frame_number, const RegionList& regions, const cv::Rect owner_location, BagDescriptor* bag);
	void UpdateKalmans(int frame_number, PersonList* known_people);
	void UpdateKalmans(int frame_number, BagList* known_bags);
	void AddCurrentProjection(const cv::Scalar& color,const cv::Mat& hsv_raw, cv::Mat* out);
	bool AreClose(const cv::Rect& lhs, const cv::Rect& rhs);
	bool AreCloseX(const cv::Rect& lhs, const cv::Rect& rhs);
	bool AreClose3d(const cv::Rect& lhs, const cv::Rect& rhs,int lhs_depth,int rhs_depth);
	cv::Scalar HSV2BGR(cv::Scalar HSV);
	cv::Scalar BGR2HSV(cv::Scalar BGR);
	int GetMeanDepth(const cv::Mat& depth_raw,const Region& upper,const Region& lower);
	int GetMeanDepth(const cv::Mat& depth_raw,const Region& region);
	bool CheckForInheldRect(const cv::Rect& rhs, const cv::Rect& lhs);
	bool Normalize(int frame_number, const cv::Mat& hsv_raw,cv::Mat* color, cv::Scalar* last_mean);
	void MaskToZero(const cv::Mat& mask, cv::Mat* input_output);
}
#endif /* UTILITY_H_ */
