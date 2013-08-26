#include "BlobDescriptor.h"
BlobDescriptor::BlobDescriptor() {
	last_seen=first_seen=-1;
	belongs_to = NO_OWNER;
	last_location = cv::Rect();
	observation_count = 1;
	relation_mean=-1;
}
BlobDescriptor::BlobDescriptor(const BlobDescriptor& rhs) {
	*this = rhs;
}
BlobDescriptor& BlobDescriptor::operator=(const BlobDescriptor& rhs) {
	image = rhs.image;
	last_seen = rhs.last_seen;
	first_seen = rhs.first_seen;
	last_location = rhs.last_location;
	rhs.upper_histogram.copyTo(upper_histogram);
	rhs.lower_histogram.copyTo(lower_histogram);
	rhs.upper_last_histogram.copyTo(upper_last_histogram);
	rhs.lower_last_histogram.copyTo(lower_last_histogram);
	observation_count = rhs.observation_count;
	relation_mean = rhs.relation_mean;
	relation_variance = rhs.relation_variance;
	belongs_to = rhs.belongs_to;
	last_relation = rhs.last_relation;
	//copy the list
	history = rhs.history;
	filter = rhs.filter;
	upper_hue_plane = rhs.upper_hue_plane;
	lower_hue_plane = rhs.lower_hue_plane;
	moments_ = rhs.moments_;
	return *this;
}
std::ostream& BlobDescriptor::operator << (std::ostream& out) {
	out <<"First seen: "<<first_seen<<"Last seen: "<<last_seen<<" at: "<<last_location<<" belongs to: "<<belongs_to;
	return out;
}
void BlobDescriptor::update(const BlobDescriptor& rhs) {
	image = rhs.image;
	last_seen = rhs.last_seen;
	//don't update first_seen
	last_location = rhs.last_location;
//	rhs.upper_histogram.copyTo(upper_histogram);
//	rhs.lower_histogram.copyTo(lower_histogram);
	if(upper_histogram.size()==rhs.upper_histogram.size() &&
	   lower_histogram.size()==rhs.lower_histogram.size() ) {
		upper_histogram = (upper_histogram*0.8)+(rhs.upper_histogram*0.2);
		lower_histogram = (lower_histogram*0.8)+(rhs.lower_histogram*0.2);
		lower_last_histogram = rhs.lower_histogram;
		upper_last_histogram = rhs.upper_histogram;
	}
	else {
		rhs.upper_histogram.copyTo(upper_histogram);
		rhs.lower_histogram.copyTo(lower_histogram);
	}
///		belongs_to = rhs.belongs_to;
	//add the location to the internal list
	HistoryDescriptor temp;
	temp.position = last_location;
	history[last_seen] = temp;
	upper_hue_plane = rhs.upper_hue_plane;
	lower_hue_plane = rhs.lower_hue_plane;
	moments_ = rhs.moments_;

	//average relations
	if(relation_mean==-1) {
		relation_mean = last_relation;
		relation_variance = last_relation*3;
	}
/*  The real formulae:
	double x1 = sqrt(relation_variance*(observation_count-1))+relation_mean;
	relation_variance = (pow(x1-relation_mean,2)+pow(last_relation-relation_mean,2))/observation_count;
*/
	//optimized version:
	double x1 = relation_variance*(observation_count);
	relation_variance = (x1+pow(last_relation-relation_mean,2))/(observation_count+1);
	relation_mean = (relation_mean*observation_count+last_relation)/(++observation_count);
}
void BlobDescriptor::init_kalman() {
	filter.init(last_location);
}
void BlobDescriptor::update_kalman(int frame_number) {
	//if we have recent location data for this blob
	if(last_seen==frame_number) {
		filter.update(&last_location);
	}
	else {
		//if this is a person or recently moving bag
		if((belongs_to == NO_OWNER) || ((frame_number-last_seen) < 4)) {
			filter.update(NULL);
		}

	}
}
