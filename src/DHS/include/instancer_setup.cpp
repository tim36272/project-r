#include "event.cpp"
#include "InstanceGrabber.h"

void SetupPossabilities(int event_index,
						int person_one_index,
						int person_two_index,
						int bag_index,
						const utility::ColorPairListType& people_colors_,
						const utility::ColorSingleListType& bag_colors_,
						std::vector<InstanceGrabber>* instances) {
	//verify the request makes sense
	if(event_index==EVENT_ONE_PERSON_SEARCH) {
		if(!(people_colors_.size()>0)) {
			return;
		}
	}
	else if(event_index==EVENT_ONE_PERSON_ABANDON) {
		if(!(people_colors_.size()>0 && bag_colors_.size()>0)) {
			return;
		}
	}
	else if(event_index==EVENT_TWO_PERSON_SEARCH) {
		if(!(people_colors_.size()>=2)) {
			return;
		}
	}
	else if(event_index==EVENT_TWO_PERSON_STEAL || event_index==EVENT_TWO_PERSON_EXCHANGE) {
		if(!(people_colors_.size()>=2 && bag_colors_.size()>0)) {
			return;
		}
	}

	//for an index which is EVENT_ANY call the function recursively
	if(event_index==EVENT_ANY) {
		SetupPossabilities(EVENT_ONE_PERSON_SEARCH,person_one_index,EVENT_NOT_USED,EVENT_NOT_USED,people_colors_,bag_colors_,instances);
		if(bag_colors_.size()>0) {
			SetupPossabilities(EVENT_ONE_PERSON_ABANDON,person_one_index,EVENT_NOT_USED,bag_index,people_colors_,bag_colors_,instances);
		}
		if(people_colors_.size()>=2) {
			SetupPossabilities(EVENT_TWO_PERSON_SEARCH,person_one_index,person_two_index,EVENT_NOT_USED,people_colors_,bag_colors_,instances);
			if(bag_colors_.size()>0) {
				SetupPossabilities(EVENT_TWO_PERSON_STEAL,person_one_index,person_two_index,bag_index,people_colors_,bag_colors_,instances);
				SetupPossabilities(EVENT_TWO_PERSON_EXCHANGE,person_one_index,person_two_index,bag_index,people_colors_,bag_colors_,instances);
			}
		}
	}
	else if(event_index==EVENT_ALL_ONE_PERSON_EVENTS) {
		SetupPossabilities(EVENT_ONE_PERSON_SEARCH,person_one_index,EVENT_NOT_USED,EVENT_NOT_USED,people_colors_,bag_colors_,instances);
		if(bag_colors_.size()>0) {
			SetupPossabilities(EVENT_ONE_PERSON_ABANDON,person_one_index,EVENT_NOT_USED,bag_index,people_colors_,bag_colors_,instances);
		}
	}
	else if(event_index==EVENT_ALL_TWO_PERSON_EVENTS) {
		if(people_colors_.size()>=2) {
			SetupPossabilities(EVENT_TWO_PERSON_SEARCH,person_one_index,person_two_index,EVENT_NOT_USED,people_colors_,bag_colors_,instances);
			if(bag_colors_.size()>0) {
				SetupPossabilities(EVENT_TWO_PERSON_STEAL,person_one_index,person_two_index,bag_index,people_colors_,bag_colors_,instances);
				SetupPossabilities(EVENT_TWO_PERSON_EXCHANGE,person_one_index,person_two_index,bag_index,people_colors_,bag_colors_,instances);
			}
		}
	}
	else if(person_one_index==EVENT_ANY) {
		for(uint person_index=0;person_index<people_colors_.size();person_index++) {
			SetupPossabilities(event_index,person_index,person_two_index,bag_index,people_colors_,bag_colors_,instances);
		}
	}
	else if(person_two_index==EVENT_ANY) {
		for(uint person_index=0;person_index<people_colors_.size();person_index++) {
			SetupPossabilities(event_index,person_one_index,person_index,bag_index,people_colors_,bag_colors_,instances);
		}
	}
	else if(bag_index==EVENT_ANY) {
		for(uint bag_hopper=0;bag_hopper<people_colors_.size();bag_hopper++) {
			SetupPossabilities(event_index,person_one_index,person_two_index,bag_hopper,people_colors_,bag_colors_,instances);
		}
	}
	else if(person_one_index!=person_two_index) {
		(*instances).push_back(InstanceGrabber(event_index,
											   people_colors_[person_one_index][0],
											   people_colors_[person_one_index][1],
											   (person_two_index!=EVENT_NOT_USED) ? people_colors_[person_two_index][0] : cv::Scalar(0),
											   (person_two_index!=EVENT_NOT_USED) ? people_colors_[person_two_index][1] : cv::Scalar(0),
											   (bag_index!=EVENT_NOT_USED) ? bag_colors_[bag_index] : cv::Scalar(0),
											   person_one_index,
											   person_two_index,
											   bag_index));
	}
}
