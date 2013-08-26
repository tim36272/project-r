#ifndef TIM_SRC_UTILITIES_CPP
#define TIM_SRC_UTILITIES_CPP

//histogram calculation
#define R_HUE 0
#define R_SAT 1

#include <ros/console.h>
#include <iostream>
#include "BlobDescriptor.h"

//Constants for histogram comparison
static const double kMinChisqrStrongRelation=		15;
static const double kMinCorrelStrongRelation=		0.7;
static const double kMinIntersectStrongRelation=	15;
static const double kMinBhattacharyyaStrongRelation=1;
static const double kMinChisqrWeakRelation=			6000;
static const double kMinCorrelWeakRelation=			0.5;
static const double kMinIntersectWeakRelation=		4;
static const double kMinBhattacharyyaWeakRelation=	0.6;

//region splitting constants
static const int kMaxRegionSplitTimes = 10;
static const int kMaxRegionCombineTimes = 5;


typedef std::vector<std::vector<cv::Point> > RegionList;

namespace utility {
	//protoypes
	void DisplayHistogram(const std::string& name, const cv::MatND& histogram);
	cv::Mat ConvertHistogramToSignature(const cv::MatND& histogram);
	float CompareBlobsEMD(const BlobDescriptor& first, const BlobDescriptor& second);


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
	 * Description: divides a region if there is a weakly connected segment
	 */
	void SplitRegion(const cv::Point& mass_center, cv::Mat& region, RegionList* new_blobs) {
		RegionList temp_blobs;
		int timeout=0;

		//erode the region a maximum of kMaxRegionSplitTimes
		while(temp_blobs.size()<2 && timeout <kMaxRegionSplitTimes) {
			temp_blobs.clear();
			cv::Mat temp_region;
			//erode the region and refind blobs in hopes of getting two or more blobs
			cv::erode(region,region,cv::Mat());
			region.copyTo(temp_region);
			cv::findContours(temp_region,temp_blobs,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			timeout++;
		}
		ROS_INFO_STREAM("Eroded region "<<timeout<<" times");
		//cull small regions
		for(int blob_index=0;blob_index<temp_blobs.size();blob_index++) {
			if(temp_blobs[blob_index].size()>300) {
				new_blobs->push_back(temp_blobs[blob_index]);
			}
		}
	}
	/*
	 * Name: CombineRegions
	 * Description: combines regions if they are close to eachother
	 */
	void CombineRegions(cv::Mat& region, RegionList* new_blobs) {
		RegionList temp_blobs;
		int timeout=0;

		//dilate the region a maximum of kMaxRegionSplitTimes
		do {
			temp_blobs.clear();
			cv::Mat temp_region;
			//dilate the region and re-find blobs in hopes of getting fewer blobs
			cv::dilate(region,region,cv::Mat());
			region.copyTo(temp_region);
			cv::findContours(temp_region,temp_blobs,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			timeout++;
		} while(temp_blobs.size()>1 && timeout <kMaxRegionCombineTimes);

		ROS_INFO_STREAM("Dilated region "<<timeout<<" times");
		ROS_INFO_STREAM("There are "<<temp_blobs.size()<<" separate regions left");
		//add all regions to the new_blobs
		for(int blob_index=0;blob_index<temp_blobs.size();blob_index++) {
			new_blobs->push_back(temp_blobs[blob_index]);
		}
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

	bool AreColliding(const cv::Rect& lhs, const cv::Rect& rhs) {
		cv::Point lhs_center(lhs.x+lhs.width/2,lhs.y+lhs.height/2);
		cv::Point rhs_center(rhs.x+rhs.width/2,rhs.y+rhs.height/2);

		int horizontal_distance = abs((lhs.x+lhs.width/2) - (rhs.x+rhs.width/2));
		int vertical_distance   = abs((lhs.y+lhs.height/2) - (rhs.y+rhs.height/2));

		int horizontal_space = lhs.width/2+rhs.width/2;
		int vertical_space   = lhs.height/2+rhs.height/2;

		if((horizontal_distance < horizontal_space) && vertical_distance < vertical_space) return true;
		else return false;
	}

	void ComputeHistogram(cv::Mat& image, cv::MatND* histogram, const cv::Mat& mask) {

		 /// Using 30 bins for hue and 32 for saturation
		  int h_bins = 30; int s_bins = 32;
		  int hist_size[] = { h_bins, s_bins };

		  // hue varies from 0 to 256, saturation from 0 to 180
		  float h_ranges[] = { 0, 180 };
		  float s_ranges[] = { 0, 255 };

		  const float* ranges[] = { h_ranges, s_ranges };

		  // Use the o-th and 1-st channels
		  int channels[] = { 0, 1 };

		//compute
		cv::calcHist(&image,1,channels,mask,*histogram,2,hist_size,ranges,true,false);
		cv::normalize(*histogram,*histogram,0,0.999,cv::NORM_MINMAX,-1,cv::Mat());
	}

	cv::Mat MakeBinaryMask(const cv::Mat& image) {
		cv::Mat out;
	    cv::cvtColor(image, out, CV_BGR2GRAY);
	    cv::threshold(out,out,0,255,cv::THRESH_BINARY);
	    return out;
	}

	void ComputeBackProjection( const cv::Mat& color_raw, const BlobDescriptor& blob , cv::MatND* back_projection ) {
		cv::Mat color_hsv;
		cv::cvtColor(color_raw,color_hsv,CV_BGR2HSV);
		/*cv::Mat mask1,mask2;
		//make a mask to suppress low saturation
		cv::threshold(planes[1],mask1,60,255,cv::THRESH_BINARY);
		//make a mask to suppress bright pixels
		cv::threshold(planes[2],mask2,170,255,cv::THRESH_BINARY_INV);
		mask1 = mask1 & mask2;*/
		cv::MatND upper_back_projection,lower_back_projection;

		  // hue varies from 0 to 256, saturation from 0 to 180
		  float h_ranges[] = { 0, 180 };
		  float s_ranges[] = { 0, 255 };
		  int channels[] = { 0, 1 };

		  const float* ranges[] = { h_ranges, s_ranges };
		cv::calcBackProject(&color_hsv,1,channels,blob.upper_histogram,upper_back_projection,ranges,1,true);
		cv::calcBackProject(&color_hsv,1,channels,blob.lower_histogram,lower_back_projection,ranges,1,true);
		*back_projection = upper_back_projection | lower_back_projection;
	}

	bool CheckForStrongRelation(double relation,double relation_mean, double relation_variance, int method) {
		if(method == CV_COMP_CHISQR) {
			std::cout<<"relation: "<<relation<<" mean: "<<relation_mean<<" variance: "<<relation_variance<<std::endl;
			if(relation_mean==-1) return true;
			return relation < (relation_mean+relation_variance);
		}
		else if (method == CV_COMP_CORREL) {
			return relation > (relation_mean-relation_variance);
		}
		else if (method== CV_COMP_INTERSECT){
			return relation > (relation_mean-relation_variance);
		}
		else {
			return relation < (relation_mean+relation_variance);
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

	double CompareBlobs(BlobDescriptor& first,BlobDescriptor& second,int method,double& real_relation) {
		//TODO: change how updating from last histogram this all works so that these can be const
		/////test
//		double EMD = CompareBlobsEMD(first,second);
		////end test
		double lower_relation,upper_relation;
		if(method == CV_COMP_CHISQR) {
			upper_relation = compareHist(first.upper_histogram,second.upper_histogram,CV_COMP_CHISQR);
			std::cout<<"Upper relation: "<<upper_relation<<std::endl;
			lower_relation = compareHist(first.lower_histogram,second.lower_histogram,CV_COMP_CHISQR);
			std::cout<<"lower relation: "<<lower_relation<<std::endl;
			real_relation = (lower_relation+upper_relation)/2;
			//if this is in the rejection region, check some more things
			if(!CheckForStrongRelation((upper_relation+lower_relation)/2,first.relation_mean,first.relation_variance,method)) {
				std::cout<<"in possible rejection region, checking last histograms"<<std::endl;
				//we are probably in rejection region
				if(first.upper_last_histogram.data && first.lower_last_histogram.data) {
					double new_upper_relation = compareHist(first.upper_last_histogram,second.upper_histogram,CV_COMP_CHISQR);
					double new_lower_relation = compareHist(first.lower_last_histogram,second.lower_histogram,CV_COMP_CHISQR);

					if(upper_relation>new_upper_relation) {
						if(new_upper_relation < upper_relation) {
							upper_relation = new_upper_relation;
							first.upper_histogram = first.upper_last_histogram;
							std::cout<<"New upper relation: "<<new_upper_relation<<std::endl;
						}
					}
					if(lower_relation > new_lower_relation) {
						if(new_lower_relation < lower_relation) {
							lower_relation = new_lower_relation;
							first.lower_histogram = first.lower_last_histogram;
							std::cout<<"New lower relation: "<<new_lower_relation<<std::endl;
						}
					}
				}
				else {
					std::cout<<"There was no last histogram"<<std::endl;
				}

				//check again and do EMD if fails
				if(!CheckForStrongRelation((upper_relation+lower_relation)/2,first.relation_mean,first.relation_variance,method)) {
					//calculate EMD as a last resort
					if(upper_relation>lower_relation) {
						float new_upper_relation = 0;

						cv::Mat first_upper_signature = ConvertHistogramToSignature(first.upper_histogram);
						cv::Mat second_upper_signature = ConvertHistogramToSignature(second.upper_histogram);

						new_upper_relation = cv::EMD(first_upper_signature,second_upper_signature,CV_DIST_L1);
						std::cout<<"Upper EMD: "<<new_upper_relation<<std::endl;
					}
					else {
						float new_lower_relation = 0;

						cv::Mat first_upper_signature = ConvertHistogramToSignature(first.upper_histogram);
						cv::Mat second_upper_signature = ConvertHistogramToSignature(second.upper_histogram);

						new_lower_relation = cv::EMD(first_upper_signature,second_upper_signature,CV_DIST_L1);
						std::cout<<"Lower EMD: "<<new_lower_relation<<std::endl;
					}

				}
			}
			if(upper_relation+lower_relation > 100) {
/*				std::ofstream fout;
				fout.clear();
				fout.open("different histograms");
				fout<<std::endl<<std::endl<<"known upper histogram: "<<first.upper_histogram.size()<<std::endl<<first.upper_histogram<<std::endl;
				fout<<std::endl<<std::endl<<"known lower histogram: "<<first.lower_histogram.size()<<std::endl<<first.lower_histogram<<std::endl;
				fout<<std::endl<<std::endl<<"new upper histogram: "<<second.upper_histogram.size()<<std::endl<<second.upper_histogram<<std::endl;
				fout<<std::endl<<std::endl<<"new lower histogram: "<<second.lower_histogram.size()<<std::endl<<second.lower_histogram<<std::endl;
				fout.close();
				std::stringstream name;
				name <<upper_relation+lower_relation<<"known upper";
				DisplayHistogram(name.str(),first.upper_histogram);
				name<<"i";
				imshow(name.str(),first.image);
				name.str(""); name <<upper_relation+lower_relation<<"known lower";
				DisplayHistogram(name.str(),first.lower_histogram);
				name.str(""); name <<upper_relation+lower_relation<<"new upper";
				DisplayHistogram(name.str(),second.upper_histogram);
				name<<"i";
				imshow(name.str(),second.image);
				name.str(""); name <<upper_relation+lower_relation<<"new lower";
				DisplayHistogram(name.str(),second.lower_histogram);
				cv::waitKey(1);*/
			}
		}
		else if (method == CV_COMP_CORREL) {
			upper_relation = compareHist(first.upper_histogram,second.upper_histogram,CV_COMP_CORREL);
			lower_relation = compareHist(first.lower_histogram,second.lower_histogram,CV_COMP_CORREL);
			real_relation = (lower_relation+upper_relation)/2;
		}
		else if (method== CV_COMP_INTERSECT){
			upper_relation = compareHist(first.upper_histogram,second.upper_histogram,CV_COMP_INTERSECT);
			lower_relation = compareHist(first.lower_histogram,second.lower_histogram,CV_COMP_INTERSECT);
			real_relation = (lower_relation+upper_relation)/2;
		}
		else {
			upper_relation = compareHist(first.upper_histogram,second.upper_histogram,CV_COMP_BHATTACHARYYA);
			lower_relation = compareHist(first.lower_histogram,second.lower_histogram,CV_COMP_BHATTACHARYYA);
			real_relation = (lower_relation+upper_relation)/2;
		}
		return (upper_relation+lower_relation)/2;
	}

	void PrintHistogramComparison(const cv::MatND& rhs, const cv::MatND& lhs) {
		if(rhs.size!=lhs.size) {
			ROS_ERROR("Histograms were different sizes");
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

	cv::Rect CalculateSharedRect(const cv::Rect& lhs, const cv::Rect& rhs) {
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
			return cv::Rect();
		}

		//compute the rect
		cv::Rect shared(upper_left.x,upper_left.y,lower_right.x-upper_left.x,lower_right.y-upper_left.y);
		return shared;
	}

	void WriteMap(std::ofstream& out, std::map<int,HistoryDescriptor> history) {
		out<<history.size()<<std::endl;
		std::map<int,HistoryDescriptor>::iterator it=history.begin();
		for (int map_index=0; map_index<history.size(); map_index++) {
		    out << it->first << "=" << it->second << std::endl;
		    it++;
		}
		out<<std::endl;
	}

	void LookForBagUsingBackProjection(const cv::Mat& color_raw, const BlobDescriptor& blob) {
		cv::MatND histogram;
		utility::ComputeBackProjection(color_raw, blob , &histogram );
		imshow("bag histogram",histogram);
		//get contours from the back projection
		cv::Mat bag_foreground;
		cv::threshold(histogram,histogram,180,255,cv::THRESH_BINARY);
		//make a mask for guessing where the bag is
		cv::Mat known_bag_mask(histogram.size(),CV_8UC1,cv::Scalar(0));
		cv::rectangle(known_bag_mask,blob.filter.bounding_rect_estimate(),cv::Scalar(255),-1,8);
		histogram.copyTo(bag_foreground,known_bag_mask);
		//process the frame
		cv::dilate(bag_foreground,bag_foreground,cv::Mat());
		cv::dilate(bag_foreground,bag_foreground,cv::Mat());
		cv::erode(bag_foreground,bag_foreground,cv::Mat());
		cv::erode(bag_foreground,bag_foreground,cv::Mat());

		//make a mat with the contours on it
		std::vector<std::vector<cv::Point> > bag_contours;
		cv::findContours(bag_foreground,bag_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		cv::Mat bag_contour_mat(bag_foreground.size(),CV_8UC3,cv::Scalar(0));
		for(unsigned int i=0;i<bag_contours.size();i++) {
			if(cv::contourArea(bag_contours[i])>300) {
				ROS_INFO_STREAM("Region size: "<<cv::contourArea(bag_contours[i])<<" possible size: "<<blob.filter.bounding_rect_estimate().area());
				//color them and encode their region number into blue and green
				int blue = i%256;
				int green = (i/256)%256;
				cv::Scalar color(blue,green,rand()%256);
				//draw the contour on the Mat
				cv::drawContours(bag_contour_mat,bag_contours,i,color,1);
			}
		}
	}

	void FindCollisionCandidates(const std::vector<BlobDescriptor>& blobs, std::vector<std::vector<int> >* collision_candidates) {
		for(int blob_index=0;blob_index<blobs.size();blob_index++) {
			std::vector<int> candidates_with_this_blob;
			for(int second_blob_index=blob_index+1;second_blob_index<blobs.size();second_blob_index++) {
				if(utility::AreColliding(blobs[blob_index].filter.bounding_rect_estimate(),
									 blobs[second_blob_index].filter.bounding_rect_estimate()) ) {
					candidates_with_this_blob.push_back(second_blob_index);
				}
			}
			(*collision_candidates).push_back(candidates_with_this_blob);
		}
	}

	bool RegionOverlapsBlob(const BlobDescriptor& blob, const std::vector<cv::Point> region) {
		return utility::AreColliding(blob.filter.bounding_rect_estimate(),cv::boundingRect(region));
	}

	void GetOverlappingRegions(const std::vector<std::vector<int> >& collision_candidates,
							   const RegionList& regions,
							   const std::vector<BlobDescriptor> known_blobs,
							   std::vector<std::vector<int> >* overlapping_regions) {
		for(int first_blob_index=0;first_blob_index<collision_candidates.size();first_blob_index++) {
				std::vector<int> these_overlapping_regions;
				for(int second_blob_index=0;second_blob_index<collision_candidates[first_blob_index].size();second_blob_index++) {
					for(int region_index=0;region_index<regions.size();region_index++) {
						if(utility::RegionOverlapsBlob(known_blobs[first_blob_index],(regions)[region_index]) ||
						   utility::RegionOverlapsBlob(known_blobs[second_blob_index],(regions)[region_index])	) {
							these_overlapping_regions.push_back(region_index);
						}
					}
				}
				overlapping_regions->push_back(these_overlapping_regions);
			}
	}

	void ReplaceRegions(const cv::Mat& color_raw, const std::vector<int>& old_regions,const RegionList& new_regions,RegionList* regions) {
		//erase the old regions
		for(int old_region_index=old_regions.size();old_region_index>0;old_region_index--) {

			regions->erase(regions->begin()+old_regions[old_region_index]);
		}

		//add new ones
		//draw all them on one mat
		cv::Mat temp(color_raw.size(),CV_8UC1,cv::Scalar(0));
		cv::drawContours(temp,new_regions,-1,cv::Scalar(255),-1,8);
		cv::drawContours(temp,*regions,-1,cv::Scalar(255),-1,8);

		RegionList new_out;
		cv::findContours(temp,new_out,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		regions->swap(new_out);
	}

	void DisplayHistogram(const std::string& name, const cv::MatND& histogram) {
	    double maxVal=0;
	    minMaxLoc(histogram, 0, &maxVal, 0, 0);
	    int hbins = 30, sbins = 32;
	    int scale = 10;
	    cv::Mat histImg = cv::Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

	    for( int h = 0; h < hbins; h++ )
	        for( int s = 0; s < sbins; s++ )
	        {
	            float binVal = histogram.at<float>(h, s);
	            double intensity = cvRound(binVal*255./maxVal);
	            cv::rectangle( histImg, cv::Point(h*scale, s*scale),
	                        cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
	                        cv::Scalar::all(intensity),
	                        CV_FILLED );
	        }
	    cv::namedWindow( name, 1 );
	    imshow( name, histImg );

	}

	cv::Mat ConvertHistogramToSignature(const cv::MatND& histogram) {
	    int hbins = 30, sbins = 32;
		cv::Mat signature(hbins*sbins,3,CV_32FC1);
	    for(int h=0;h<hbins;h++) {
	    	for(int s=0;s<sbins;s++) {
	    		float bin_value = histogram.at<float>(h,s);
	    		signature.at<float>(h*sbins+s,0) = bin_value;
	    		signature.at<float>(h*sbins+s,1) = h;
	    		signature.at<float>(h*sbins+s,2) = s;
	    	}
	    }
	    std::ofstream fout;
	    fout.clear();
	    fout.open("signature");
	    fout <<signature;
	    fout.close();
	    return signature;
	}

	float CompareBlobsEMD(const BlobDescriptor& first, const BlobDescriptor& second) {
		//create both signatures
		float upper_relation = 0;
		float lower_relation = 0;

		cv::Mat first_upper_signature = ConvertHistogramToSignature(first.upper_histogram);
		cv::Mat first_lower_signature = ConvertHistogramToSignature(first.lower_histogram);
		cv::Mat second_upper_signature = ConvertHistogramToSignature(second.upper_histogram);
		cv::Mat second_lower_signature = ConvertHistogramToSignature(second.lower_histogram);

		upper_relation += cv::EMD(first_upper_signature,second_upper_signature,CV_DIST_L1);
		lower_relation += cv::EMD(first_lower_signature,second_lower_signature,CV_DIST_L1);
		std::cout<<"Upper EMD: "<<upper_relation<<std::endl;
		std::cout<<"Lower EMD: "<<lower_relation<<std::endl;
		return (upper_relation+lower_relation)/2.;
	}
}
#endif
