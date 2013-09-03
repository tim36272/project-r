#include "BlobDescriptor.h"

PersonDescriptor::PersonDescriptor() {
	last_seen=first_seen=-1;
	observation_count = 1;
}
void PersonDescriptor::init_kalman() {
	filter.init(last_location);
}
void PersonDescriptor::update_kalman(int frame_number) {
	//if we have recent location data for this blob
	if(last_seen==frame_number) {
		filter.update(&last_location);
	}
	else if(frame_number-last_seen < 10) {
		filter.update(NULL);
	}
}
PersonDescriptor::PersonDescriptor(const PersonDescriptor& rhs) {
	*this = rhs;
}

PersonDescriptor& PersonDescriptor::operator=(const PersonDescriptor& rhs) {
	//this needs updating
	last_seen = rhs.last_seen;
	first_seen = rhs.first_seen;
	observation_count = rhs.observation_count;
	last_location = rhs.last_location;
	color = rhs.color;
	last_upper_location = rhs.last_upper_location;
	last_lower_location = rhs. last_lower_location;

	//copy the list
	history = rhs.history;
	filter = rhs.filter;
	return *this;
}
/////////////// Bag //////////////////////
BagDescriptor::BagDescriptor() {
	last_seen=first_seen=-1;
	observation_count = 1;
}
void BagDescriptor::init_kalman() {
	filter.init(last_location);
}
void BagDescriptor::update_kalman(int frame_number) {
	//if we have recent location data for this blob
	if(last_seen==frame_number) {
		filter.update(&last_location);
	}
	else {// if(frame_number-last_seen < 4) {
		filter.update(NULL);
	}
}
BagDescriptor::BagDescriptor(const BagDescriptor& rhs) {
	*this = rhs;
}

BagDescriptor& BagDescriptor::operator=(const BagDescriptor& rhs) {
	last_seen = rhs.last_seen;
	first_seen = rhs.first_seen;
	observation_count = rhs.observation_count;
	last_location = rhs.last_location;
	color = rhs.color;

	//copy the list
	history = rhs.history;
	filter = rhs.filter;
	belongs_to = rhs.belongs_to;
	return *this;
}
