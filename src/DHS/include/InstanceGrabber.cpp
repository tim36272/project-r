/*
 * InstanceGrabber.cpp
 *
 *  Created on: Sep 2, 2013
 *      Author: tim
 */
#include "InstanceGrabber.h"

InstanceGrabber::InstanceGrabber() {
	assert(false);
	setup_done_ = false;
	looking_for_start_ = true;
}

void InstanceGrabber::Setup(uint event_code,
		 const cv::Scalar person_one_upper_color,
		 const cv::Scalar person_one_lower_color,
		 const cv::Scalar  person_two_upper_color,
		 const cv::Scalar  person_two_lower_color,
		 const cv::Scalar bag_color,
		 int person_one_source_index,
		 int person_two_source_index,
		 int bag_source_index) {

	event_code_ = event_code;
	person_one_upper_color_ = person_one_upper_color;
	person_one_lower_color_ = person_one_lower_color;
	person_two_upper_color_ = person_two_upper_color;
	person_two_lower_color_ = person_two_lower_color;
	bag_color_ = bag_color;
	std::cout<<"event code: "<<event_code<<std::cout;
	assert(event_code>=EVENT_ONE_PERSON_SEARCH && event_code<=EVENT_TWO_PERSON_EXCHANGE);
	person_one_source_index_ = person_one_source_index;
	person_two_source_index_ = person_two_source_index;
	bag_source_index_ = bag_source_index;


	looking_for_start_ = true;
	missing_person_buffer_ = 10;
	setup_done_ = true;
}
InstanceGrabber::InstanceGrabber(uint objective_code,
		 const cv::Scalar person_one_upper_color,
		 const cv::Scalar person_one_lower_color,
		 const cv::Scalar  person_two_upper_color,
		 const cv::Scalar  person_two_lower_color,
		 const cv::Scalar bag_color,
		 int person_one_source_index,
		 int person_two_source_index,
		 int bag_source_index) {
	Setup(objective_code,person_one_upper_color,person_one_lower_color,person_two_upper_color,person_two_lower_color,bag_color,person_one_source_index,person_two_source_index,bag_source_index);
}

void InstanceGrabber::Update(int frame_number,const PersonList& people,const BagList& bags) {
	if(!setup_done_) return;
	frame_number_ = frame_number;
	switch(event_code_) {
	case EVENT_ONE_PERSON_SEARCH:
		UpdateOnePersonSearch(people);
		break;
	case EVENT_ONE_PERSON_ABANDON:
		UpdateOnePersonAbandon(people,bags);
		break;
	case EVENT_TWO_PERSON_SEARCH:
		UpdateTwoPersonSearch(people);
		break;
	case EVENT_TWO_PERSON_STEAL:
		UpdateTwoPersonSteal(people,bags);
		break;
	case EVENT_TWO_PERSON_EXCHANGE:
		UpdateTwoPersonExchange(people,bags);
		break;
	}
}


std::ostream& operator <<(std::ostream& out, const InstanceGrabber& rhs) {
	if(!rhs.setup_done_) return out;
	for(uint instance_index=0;instance_index<rhs.instances_.size();instance_index++) {
		out<<std::endl<<std::endl;
		out<<"Start: "<<rhs.instances_[instance_index][START_TIME];
		out<<" | End: "<<(rhs.instances_[instance_index][END_TIME]!=-1 ? rhs.instances_[instance_index][END_TIME] : rhs.frame_number_) ;
	}
	return out;
}
int InstanceGrabber::GetFirstPersonIndex(const PersonList& people) {
	for(int person_index=0;person_index<people.size();person_index++) {
		if(people[person_index].color[UPPER_COLOR] == person_one_upper_color_ &&
		   people[person_index].color[LOWER_COLOR] == person_one_lower_color_) {
			return person_index;
		}
	}
	return NOT_FOUND;
}
int InstanceGrabber::GetSecondPersonIndex(const PersonList& people) {
	for(int person_index=0;person_index<people.size();person_index++) {
		if(people[person_index].color[UPPER_COLOR] == person_two_upper_color_ &&
		   people[person_index].color[LOWER_COLOR] == person_two_lower_color_) {
			return person_index;
		}
	}
	return NOT_FOUND;
}
int InstanceGrabber::GetBagIndex(const BagList& bags) {
	for(int bag_index=0;bag_index<bags.size();bag_index++) {
		if(bags[bag_index].color == bag_color_) {
			return bag_index;
		}
	}
	return NOT_FOUND;
}

bool InstanceGrabber::EventInProgress() const {
	 return !looking_for_start_;
}

void InstanceGrabber::UpdateOnePersonSearch(const PersonList& people) {
	//get index of the target person
	int person_index = GetFirstPersonIndex(people);
	if(looking_for_start_) {
		if(person_index==NOT_FOUND) {
			return; //they haven't been seen at all yet
		}
		else { // they were at one point visible, see if that is now
			if(people[person_index].last_seen==frame_number_) {
				//then this is the first time we are seeing them for this instance
				utility::Pair temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = NOT_FOUND;
				instances_.push_back(temp);
				looking_for_start_ = false;
			}
		}


	}
	else { //looking for end
		if(person_index==NOT_FOUND) {
			//if the person is not in the list, but we are looking for the end,
			//then the master must have erased them, perhaps for a false positive
			//so erase this instance
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
		}
		else {
			//see if the last seen time was more than missing_person_buffer frames ago
			if((frame_number_ - people[person_index].last_seen) > missing_person_buffer_) {
				//the instance is over
				instances_[instances_.size()-1][END_TIME] = frame_number_-missing_person_buffer_;
				looking_for_start_ = true;
			}
		}
	}
}
void InstanceGrabber::UpdateOnePersonAbandon(const PersonList& people,const BagList& bags) {
	//get index of the target person
	int person_index = GetFirstPersonIndex(people);
	int bag_index = GetBagIndex(bags);
	if(looking_for_start_) {
		//the person and bag must be visible to start, but the person could be off camera and hence has abandoned the bag
		if(person_index==NOT_FOUND || bag_index==NOT_FOUND || bags[bag_index].last_seen!=frame_number_) {
			return;
		}
		else {
			//They have both been seen before and the bag is currently visible
			// so check if they are not close or the person is not visible(left the camera)
			cv::Rect person,bag;
			person = people[person_index].last_location;
			bag = bags[bag_index].last_location;
			if(!utility::AreClose(person,bag) || people[person_index].last_seen!=frame_number_) {
				//start instance
				utility::Pair temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = NOT_FOUND;
				instances_.push_back(temp);
				looking_for_start_ = false;
			}
			else {
				return;
			}
		}

	}
	else { //looking for end
		if(person_index==NOT_FOUND || bag_index==NOT_FOUND) {
			//if the person is not in the list, but we are looking for the end,
			//then the master must have erased them, perhaps for a false positive
			//so erase this instance
			instances_.erase(instances_.begin()+instances_.size()-1);
		}
		else {
			//check if they are now close or the bag is missing
			cv::Rect person,bag;
			person = people[person_index].last_location;
			bag = bags[bag_index].last_location;
			if(utility::AreClose(person,bag) || (frame_number_ - bags[bag_index].last_seen > missing_person_buffer_)) {
				//the instance is over
				instances_[instances_.size()-1][END_TIME] = frame_number_;
				looking_for_start_ = true;
			}
		}

	}
}
void InstanceGrabber::UpdateTwoPersonSearch(const PersonList& people) {
	int first_person_index = GetFirstPersonIndex(people);
	int second_person_index = GetSecondPersonIndex(people);
	if(first_person_index!=NOT_FOUND && second_person_index!=NOT_FOUND) {
		std::cout<<"Depths: "<<people[first_person_index].depth_position<<" , "<<people[second_person_index].depth_position<<std::endl;
	}
	if(looking_for_start_) {
		//if they are not both visible this can't start
		if(first_person_index==NOT_FOUND || second_person_index==NOT_FOUND) {
			return;
		}
		else if(people[first_person_index].last_seen!=frame_number_||
		   people[second_person_index].last_seen!=frame_number_ ||
		   !utility::AreClose3d(people[first_person_index].last_location,people[second_person_index].last_location,
		   	   	   	   	   	    people[first_person_index].depth_position,people[second_person_index].depth_position)) {
			return;
		}
		else {
			//start instance
			utility::Pair temp;
			temp[START_TIME] = frame_number_;
			temp[END_TIME] = NOT_FOUND;
			instances_.push_back(temp);
			looking_for_start_ = false;
		}
	}
	else { //looking for end
		if(first_person_index==NOT_FOUND || second_person_index==NOT_FOUND) {
			//one or both was erased by the master, assume false positive
			//abort this instance
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
			return;
		}
		else if((people[first_person_index].last_seen==frame_number_&&
		   people[second_person_index].last_seen==frame_number_) &&
		   utility::AreClose3d(people[first_person_index].last_location,people[second_person_index].last_location,
				   	   	   	 people[first_person_index].depth_position,people[second_person_index].depth_position)) {
			return;
		}
		else {
			//the instance is over
			instances_[instances_.size()-1][END_TIME] = frame_number_;
			looking_for_start_ = true;
		}
	}
}
void InstanceGrabber::UpdateTwoPersonSteal(const PersonList& people,const BagList& bags) {
	int first_person_index=GetFirstPersonIndex(people);
	int second_person_index=GetFirstPersonIndex(people);
	int bag_index = GetBagIndex(bags);
	if(looking_for_start_) {
		//the second person and bag must be visible
		if(second_person_index==NOT_FOUND || bag_index==NOT_FOUND ||
		   people[second_person_index].last_seen!=frame_number_ ||
		   bags[bag_index].last_seen!=frame_number_) {
			return;
		}
		else if(people[first_person_index].last_seen==frame_number_){ //if the owner can currently be seen
			//the instance starts if person two is close to the bag and person one is not
			cv::Rect person_one = people[first_person_index].last_location;
			cv::Rect person_two = people[second_person_index].last_location;
			cv::Rect bag = bags[bag_index].last_location;
			if(utility::AreClose(person_two,bag) && !utility::AreClose(person_one,bag)) {
				//start instance
				utility::Pair temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = NOT_FOUND;
				instances_.push_back(temp);
				looking_for_start_ = false;
				bag_location_ = bag;
			}
		}
		else {//the owner cannot be seen
			cv::Rect person_two = people[second_person_index].last_location;
			cv::Rect bag = bags[bag_index].last_location;
			if(utility::AreClose(person_two,bag)) {
				//start instance
				utility::Pair temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = NOT_FOUND;
				instances_.push_back(temp);
				looking_for_start_ = false;
				bag_location_ = bag;
			}
		}
	}
	else { //looking for end
		if(first_person_index==NOT_FOUND || second_person_index==NOT_FOUND || bag_index==NOT_FOUND) {
			//either the person or bag was erased by the master
			//assume false positive
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
			return;
		}
		else if(((frame_number_-people[second_person_index].last_seen)>missing_person_buffer_) ||
				((frame_number_-bags[bag_index].last_seen)>missing_person_buffer_)) {
			//either the bag or person disappeared from view
			//the instance is confirmed over
			instances_[instances_.size()-1][END_TIME] = frame_number_;
			looking_for_start_ = true;
			return;
		}
		else if(utility::AreClose(people[first_person_index].last_location,bags[bag_index].last_location)){
			//the owner is now nearby, assume a false positive
			//delete the instance
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
			return;
		}
		else if(!utility::AreClose(bags[bag_index].last_location,bag_location_)) {
			//the bag has moved
			//the instance is confirmed over
			instances_[instances_.size()-1][END_TIME] = frame_number_;
			looking_for_start_ = true;
			return;
		}
		else {
			//nothing changed
			return;
		}
	}
}
void InstanceGrabber::UpdateTwoPersonExchange(const PersonList& people,const BagList& bags) {
	int first_person_index=GetFirstPersonIndex(people);
	int second_person_index=GetFirstPersonIndex(people);
	int bag_index = GetBagIndex(bags);
	if(looking_for_start_) {
		//the first, second person and bag must be visible
		if(first_person_index==NOT_FOUND || second_person_index==NOT_FOUND || bag_index==NOT_FOUND ||
		   people[second_person_index].last_seen!=frame_number_ ||
		   bags[bag_index].last_seen!=frame_number_) {
			return;
		}
		else if(people[first_person_index].last_seen==frame_number_ &&
				people[second_person_index].last_seen==frame_number_ &&
				bags[bag_index].last_seen==frame_number_) {//if they can currently be seen
			//the instance starts if all three are close to the bag
			cv::Rect person_one = people[first_person_index].last_location;
			cv::Rect person_two = people[second_person_index].last_location;
			cv::Rect bag = bags[bag_index].last_location;
			if(utility::AreClose(person_two,bag) && utility::AreClose(person_one, bag)) {
				//start instance
				utility::Pair temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = NOT_FOUND;
				instances_.push_back(temp);
				looking_for_start_ = false;
				bag_location_ = bag;
			}
		}
		else {
			return;
		}
	}
	else { //looking for end
		if(first_person_index==NOT_FOUND || second_person_index==NOT_FOUND || bag_index==NOT_FOUND) {
			//either the person or bag was erased by the master
			//assume false positive
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
			return;
		}
		else if((utility::AreClose(people[second_person_index].last_location,bags[bag_index].last_location) ||
				(frame_number_-people[second_person_index].last_seen) < missing_person_buffer_)
				&&
				(!utility::AreClose(people[first_person_index].last_location,bags[bag_index].last_location) ||
				frame_number_-people[first_person_index].last_seen > missing_person_buffer_)){
			//Either:
				//the first person is not visible or is close to the bag
				//AND
				//the owner is not close to the bag or the owner cannot be seen
			//this needs some work because what if both people disappear?
			//the instance is confirmed over
			instances_[instances_.size()-1][END_TIME] = frame_number_;
			looking_for_start_ = true;
			return;
		}
		else if(!utility::AreClose(people[second_person_index].last_location,bags[bag_index].last_location)) {
			//the second person and bag are not close to eachother
			//the exchange didn't happen
			instances_.erase(instances_.begin()+instances_.size()-1);
			looking_for_start_ = true;
			return;
		}
		else {
			//nothing changed
			return;
		}
	}
}




















//////////////junk code //////////////////////
/*
void InstanceGrabber::Update(int frame_number,const PersonList& people) {
	if(!setup_done_) return;
	frame_number_ = frame_number;
	switch(objective_code_) {
	case R_ONE_PERSON_TRACK:
		UpdateOnePersonTrack(people);
		break;
	case R_TWO_PERSON_MEET:
		UpdateTwoPersonMeet(people);
		break;
	}
}

void InstanceGrabber::UpdateOnePersonTrack(const PersonList& people) {
	if(looking_for_start_) {
		//see if the person is in the list
		for(uint person_index=0;person_index<people.size();person_index++) {
			//if the person was seen this frame
			if(people[person_index].last_seen==frame_number_) {
				//if it is the person we are looking for
				if(people[person_index].color[0]==person_1_upper_ &&
				   people[person_index].color[1]==person_1_lower_) {
					utility::Pair_<uint> temp;
					temp[START_TIME] = frame_number_;
					temp[END_TIME] = frame_number_+1; //this is just for fun, it should never matter
					instances_.push_back(temp);
					looking_for_start_ = false;
					std::cout<<"Starting instance of one person track"<<std::endl;
				}
			}
		}
	}
	else {
		//see if the person is in the list
		for(uint person_index=0;person_index<people.size();person_index++) {
			//if the person was seen this frame
			if(people[person_index].color[0]==person_1_upper_ &&
			   people[person_index].color[1]==person_1_lower_) {
				//if it is the person we are looking for
				if(people[person_index].last_seen==(frame_number_-missing_person_buffer_)) {
					instances_[instances_.size()-1][END_TIME] = frame_number_;
					looking_for_start_ = true;
					std::cout<<"Ending instance"<<std::endl;
				}
			}
		}
	}
}
void InstanceGrabber::UpdateOnePersonBagUnattended(const PersonList& people,const BagList& bags) {
	if(looking_for_start_) {
		std::vector<int> visible_people;
		//see if both the person and bag are visible
		for(uint person_index=0;person_index<people.size();person_index++) {
			if(people[person_index].last_seen==frame_number_) {
				if(people[person_index].color[0]==person_1_upper_ &&
				   people[person_index].color[1]==person_1_lower_) {
					visible_people.push_back(person_index);
				}
			}
		}
		for(uint bag_index=0;bag_index<bags.size();bag_index++) {
			if(bags[bag_index].last_seen==frame_number_) {
				if(bags[bag_index].color==bag_) {
					//for each bag candidate, if the person is visible check if they are far away
					//TODO: make this work for people other than the first index
					if(people.size()>0) {
						//check if the bag and person are not close
						if(!utility::AreClose(bags[bag_index].filter.bounding_rect(),people[0].filter.bounding_rect())) {
							utility::Pair_<uint> temp;
							temp[START_TIME] = frame_number_;
							temp[END_TIME] = frame_number_+1; //this is just for fun, it should never matter
							instances_.push_back(temp);
							looking_for_start_ = false;
							std::cout<<"starting instance of one person bag unattended"<<std::endl;
						}
					}
				}
			}
		}



	}
	else {
		std::vector<int> visible_people;
		//see if both the person and bag are visible
		for(uint person_index=0;person_index<people.size();person_index++) {
			if(people[person_index].color[0]==person_1_upper_ &&
			   people[person_index].color[1]==person_1_lower_) {
				if(people[person_index].last_seen == (frame_number_-missing_person_buffer_)) {
					//this is the person and they are not in the frame
				}
				else {
					visible_people.push_back(person_index);
				}
			}
		}
		//if we get here then the person is visible so we need to check if the bag is visible and nearby
		for(uint bag_index=0;bag_index<bags.size();bag_index++) {
			if(bags[bag_index].color==bag_) {
				if(bags[bag_index].last_seen==(frame_number_-missing_person_buffer_)) {
					//the bag disappeared
					instances_[instances_.size()-1][END_TIME] = frame_number_;
					looking_for_start_ = true;
					std::cout<<"stopping instance 2"<<std::endl;
					return;
				}
				else {
					//the bag has not disappeared, so see if its owner is nearby
					if(utility::AreClose(bags[bag_index].filter.bounding_rect(),people[0].filter.bounding_rect())) {
						instances_[instances_.size()-1][END_TIME] = frame_number_;
						looking_for_start_ = true;
						std::cout<<"stopping instance 3"<<std::endl;
						return;
					}
				}
			}
		}
	}
}
void InstanceGrabber::UpdateTwoPersonBagExchange(const PersonList& people,const BagList& bags) {
	//exchange is where person 0 and 1 and the bag are close together, and then person 0 leaves. If person 0 fails to leave the exchange is cancelled
	if(looking_for_start_) {
		//TODO: make this work for arbitrary people positions
		//if there are enough people and bags
		if(people.size()>1 && bags.size()>0) {
			cv::Rect bag_location,owner_location,guest_location;

			bag_location = bags[0].filter.bounding_rect();
			owner_location = people[0].filter.bounding_rect();
			guest_location = people[1].filter.bounding_rect();

			//if the bag is close to both
			if(utility::AreClose(bag_location,owner_location) && utility::AreClose(bag_location,guest_location)) {
				//start a potential instance
				utility::Pair_<uint> temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = frame_number_+1; //this is just for fun, it should never matter
				instances_.push_back(temp);
				looking_for_start_ = false;
				std::cout<<"starting instance two person bag exchange"<<std::endl;
			}
		}
	}
	else {
		//if the bag is close to person 1 and not close to person 0
		cv::Rect bag_location,owner_location,guest_location;

		bag_location = bags[0].filter.bounding_rect();
		owner_location = people[0].filter.bounding_rect();
		guest_location = people[1].filter.bounding_rect();
		//far from person 0
		if(!utility::AreClose(bag_location,owner_location)) {
			//close to person 1
			if(utility::AreClose(bag_location,guest_location)) {
				instances_[instances_.size()-1][END_TIME] = frame_number_;
				looking_for_start_ = true;
				std::cout<<"stopping instance 1"<<std::endl;
			}
			else { //far from both
				//cancel this instance because the bag is abandoned
				instances_.erase(instances_.begin()+(instances_.size()-1));
				looking_for_start_ = true;
				std::cout<<"rejecting instance 2"<<std::endl;
			}
		}
		else { //bag and owner are close
			//if the guest leaves
			if(!utility::AreClose(bag_location,guest_location)) {
				//abort the instance because the guest walked away without the bag
				instances_.erase(instances_.begin()+(instances_.size()-1));
				looking_for_start_ = true;
				std::cout<<"rejecting instance 3"<<std::endl;
			}
		}
	}
}
void InstanceGrabber::UpdateTwoPersonBagSteal(const PersonList& people,const BagList& bags) {
	if(already_stolen_) return;
	//a theft is where person 1 is close to the bag, the bag is moving, while person 0 is not close
	cv::Rect bag_location,owner_location,guest_location;

	bag_location = bags[0].filter.bounding_rect();
	owner_location = people[0].filter.bounding_rect();
	guest_location = people[1].filter.bounding_rect();

	if(looking_for_start_) {
		if(people.size()>1 && bags.size()>0) {

			//if theif is near bag
			if(utility::AreClose(bag_location,guest_location)) {
				//and if owner is not near bag
				if(!utility::AreClose(bag_location,owner_location)) {
					//possible start of instance
					utility::Pair_<uint> temp;
					temp[START_TIME] = frame_number_;
					temp[END_TIME] = frame_number_+1; //this is just for fun, it should never matter
					instances_.push_back(temp);
					looking_for_start_ = false;
					std::cout<<"starting instance of two person bag steal"<<std::endl;

					//save the bag location
					bag_location_ = bag_location;
				}
			}
		}
	}
	else {
		//if the owner is not nearby
		if(!utility::AreClose(bag_location,owner_location)) {
			//and if the thief is nearby
			if(utility::AreClose(bag_location,guest_location)) {
				//and the bag is moving (its current location does not overlap its old location)
				if(!utility::RectsOverlap(bag_location,bag_location_)) {
					//end of instance
					instances_[instances_.size()-1][END_TIME] = frame_number_;
					looking_for_start_ = true;
					std::cout<<"stopping instance 1"<<std::endl;
					already_stolen_ = true;
				}
			}
			else { //theif left the bag, abort instance
				instances_.erase(instances_.begin()+(instances_.size()-1));
				looking_for_start_ = true;
				std::cout<<"rejecting instance 1"<<std::endl;
			}
		}
		else { //owner came back, abort instance
			instances_.erase(instances_.begin()+(instances_.size()-1));
			looking_for_start_ = true;
			std::cout<<"rejecting instance 2"<<std::endl;
		}
	}
}
void InstanceGrabber::UpdateTwoPersonMeet(const PersonList& people) {
	if(looking_for_start_) {
		int found_person_1_at = -1,found_person_2_at = -1;
		//see if the people are in the list
		for(uint person_index=0;person_index<people.size();person_index++) {
			//if the person was seen this frame
			if(people[person_index].last_seen==frame_number_) {
				//if it is the person we are looking for
				if(people[person_index].color[0]==person_1_upper_ &&
				   people[person_index].color[1]==person_1_lower_) {
					found_person_1_at = person_index;
				}
				else if(people[person_index].color[0]==person_2_upper_ &&
						people[person_index].color[1]==person_2_lower_) {
					found_person_2_at = person_index;
				}
			}
		}
		if(found_person_1_at!=-1 && found_person_2_at!=-1) {
			//both people were found. Test if they are close
			cv::Rect person_1,person_2;
			person_1 = people[found_person_1_at].filter.bounding_rect();
			person_2 = people[found_person_2_at].filter.bounding_rect();

			if(utility::AreClose(person_1,person_2)) {
				//TODO: check in the depth plane, too
				utility::Pair_<uint> temp;
				temp[START_TIME] = frame_number_;
				temp[END_TIME] = frame_number_+1; //this is just for fun, it should never matter
				instances_.push_back(temp);
				looking_for_start_ = false;
				std::cout<<"starting instance of two person meet"<<std::endl;
			}
		}
	}
	else {
		int found_person_1_at = -1,found_person_2_at = -1;
		//see if the people are in the list
		for(uint person_index=0;person_index<people.size();person_index++) {
			//if the person was seen this frame
			if(people[person_index].last_seen > (frame_number_-missing_person_buffer_)) {
				//if it is the person we are looking for
				if(people[person_index].color[0]==person_1_upper_ &&
				   people[person_index].color[1]==person_1_lower_) {
					found_person_1_at = person_index;
				}
				else if(people[person_index].color[0]==person_2_upper_ &&
						people[person_index].color[1]==person_2_lower_) {
					found_person_2_at = person_index;
				}
			}
		}
		if(found_person_1_at!=-1 && found_person_2_at!=-1) {
			//both people were found. Test if they are close
			cv::Rect person_1,person_2;
			person_1 = people[found_person_1_at].filter.bounding_rect();
			person_2 = people[found_person_2_at].filter.bounding_rect();

			if(!utility::AreClose(person_1,person_2)) {
				//TODO: check in the depth plane, too
				//they people are not close, stop the instance
				instances_[instances_.size()-1][END_TIME] = frame_number_;
				looking_for_start_ = true;
				std::cout<<"stopping instance 1"<<std::endl;
			}
		}
		else {
			//at least one of the people was not seen, stop the instance
			instances_[instances_.size()-1][END_TIME] = frame_number_;
			looking_for_start_ = true;
			std::cout<<"stopping instance 2"<<std::endl;
		}
	}
}
*/
