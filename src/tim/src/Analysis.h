#ifndef TIM_SRC_ANALYSIS_H
#define TIM_SRC_ANALYSIS_H

#include <fstream>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>

#include "BlobDescriptor.h"
#include "utilities.cpp"

typedef std::vector<std::vector<cv::Point> > RegionList;

class Analysis {
	public:
		Analysis();

		void UpdateBlobs(std::vector<BlobDescriptor>* known_blobs);
		int AddFrame(const cv::Mat &color_raw, const cv::Mat &depth_raw);
		cv::Mat depth_foreground() {return depth_foreground_;}
		cv::Mat color_foreground() {return color_foreground_;}
		cv::Mat combined_foreground() {return combined_foreground_;}


	private:
		void GetRegions(RegionList* regions);
		void CullSmallRegions(RegionList* regions);
		void SplitCollidedRegions(const std::vector<BlobDescriptor>& known_blobs, RegionList* regions);
		void FindDepthMaskContours(const cv::Mat& depth_masked,RegionList* regions);
		void MakeAMatWithOneBlob(const RegionList& regions, const cv::Mat& color_raw, int region_index,cv::Mat* image);
		void MakeNewBlobDescriptor(const RegionList& regions ,int blob_index, BlobDescriptor* blob);
		void AnalyzeRegions(const RegionList& regions,std::vector<BlobDescriptor>* current_blobs);
		void RegisterBlobs(std::vector<BlobDescriptor>& current_blobs, std::vector<BlobDescriptor>* known_blobs);
		bool RegisterMoving(BlobDescriptor& current_blob, BlobDescriptor* known_blob);
		void RegisterBags(std::vector<BlobDescriptor>* blobs);
		void IdentifyBagBlobs(std::vector<BlobDescriptor>* blobs);
		void UpdateAllKalmans(std::vector<BlobDescriptor>* blobs);
		void TrimList(std::vector<BlobDescriptor>* blobs);

		int frame_number_;
		cv::BackgroundSubtractorMOG2 color_segmentation_, depth_segmentation_;
		cv::Mat color_foreground_,depth_foreground_,combined_foreground_;
		cv::Mat color_raw_,depth_raw_;
};

#endif
