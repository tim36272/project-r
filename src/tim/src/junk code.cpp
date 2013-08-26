	void GetCloseBlobs(const cv::Rect& base_blob,int base_blob_index,const std::vector<BlobDescriptor>& known_blobs, std::vector<int>* close_blobs) {
		for(int other_blob_index;other_blob_index<known_blobs.size();other_blob_index++) {
			if(other_blob_index !=base_blob_index) {
				if(utility::AreClose(base_blob,known_blobs[other_blob_index].filter.bounding_rect_estimate())) {
					close_blobs->push_back(other_blob_index);
				}
			}
		}
	}

void Analysis::SplitCollidedRegions(const std::vector<BlobDescriptor>& known_blobs, RegionList* regions) {
	//make a vector of bounding rects for the new regions
	std::vector<cv::Rect> new_region_bounding_rects;
	for(int new_region_index=0;new_region_index<(*regions).size();new_region_index++) {
			//this adds the region even if it is too small to be interesting, which could help with recombining later
			cv::Rect temp = cv::boundingRect((*regions)[new_region_index]);
			new_region_bounding_rects.push_back(temp);
	}

	//for each known blob
	for(int blob_index=0;blob_index<known_blobs.size();blob_index++) {
		std::vector<int> regions_overlapping_known_blob;
		//check each new region
		for(int new_region_index=0;new_region_index<new_region_bounding_rects.size();new_region_index++) {
			//see if this new region overlaps the known blob
			if(utility::CheckForInheldRect(new_region_bounding_rects[new_region_index],known_blobs[blob_index].filter.bounding_rect_estimate()) ){
				//add this new region to the list
				regions_overlapping_known_blob.push_back(new_region_index);
			}
		}
		//if more than one region overlaps this blob
		if(regions_overlapping_known_blob.size()>1) {
			// combine all the regions into one mat
			// this helps with combining a series of small incorrectly separated regions into a larger region
				//draw all the contours onto one Mat
				cv::Mat combined_region_drawings(color_raw_.size(),CV_8UC1,cv::Scalar(0));
				ROS_INFO_STREAM("Drawing contours for combine");
				for(int regions_index=0;regions_index<regions_overlapping_known_blob.size();regions_index++) {
					ROS_INFO_STREAM("Region "<<regions_overlapping_known_blob[regions_index]<<" of "<<regions->size());
					cv::drawContours(combined_region_drawings,*regions,regions_overlapping_known_blob[regions_index],cv::Scalar(255),-1,8);
				}
				ROS_INFO_STREAM("Done drawing contours for combine");
				imshow("combined_regions",combined_region_drawings);
				//try to combine the regions
				RegionList combined_regions;
				utility::CombineRegions(combined_region_drawings,&combined_regions);

			// get candidates for which blobs these could be part of besides the current one by checking other known blob proximity
			std::vector<int> close_blobs;
			utility::GetCloseBlobs(known_blobs[blob_index].filter.bounding_rect_estimate(),blob_index,known_blobs,&close_blobs);

				//if the current blob is the only candidate then just make these all into one larger region
				if(close_blobs.size()==0) {
					ROS_INFO_STREAM("combined "<<regions_overlapping_known_blob.size()<<" regions into "<<combined_regions.size()<<" regions");
					//remove the input regions and add the new combined regions
					for(int overlapping_regions_index=0;overlapping_regions_index<regions_overlapping_known_blob.size();overlapping_regions_index++) {
						regions->erase(regions->begin()+regions_overlapping_known_blob[overlapping_regions_index]);
					}
					for(int combined_regions_index=0;combined_regions_index<combined_regions.size();combined_regions_index++) {
						regions->push_back(combined_regions[combined_regions_index]);
					}
					ROS_INFO_STREAM("done fixing regions");
				}
				//if there are multiple candidates, arbitrate what new regions belong where and divide them up, modifying the regions list
				else {
					ROS_INFO_STREAM("There are "<<close_blobs.size()<<" colliding regions to arbitrate");
				}
		}
	}
}
