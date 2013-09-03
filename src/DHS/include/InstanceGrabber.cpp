/*
 * InstanceGrabber.cpp
 *
 *  Created on: Sep 2, 2013
 *      Author: tim
 */
#include "InstanceGrabber.h"

InstanceGrabber::InstanceGrabber() {
	setup_done_ = false;
	looking_for_start_ = true;
	already_stolen_ = false;
}
InstanceGrabber::InstanceGrabber(int objective_code,
								 const cv::Scalar& person_1_upper,
								 const cv::Scalar& person_1_lower) {
	Setup(objective_code,person_1_upper,person_1_lower);
}
InstanceGrabber::InstanceGrabber(int objective_code,
								 const cv::Scalar& person_1_upper,
								 const cv::Scalar& person_1_lower,
								 const cv::Scalar& bag) {
	Setup(objective_code,person_1_upper,person_1_lower,bag);
}
void InstanceGrabber::Setup(int objective_code,
							 const cv::Scalar& person_1_upper,
							 const cv::Scalar& person_1_lower,
							 const cv::Scalar& person_2_upper,
							 const cv::Scalar& person_2_lower) {
	assert(objective_code==R_TWO_PERSON_MEET);
	objective_code_ = objective_code;
	person_1_upper_ = person_1_upper;
	person_1_lower_ = person_1_lower;
	person_2_upper_ = person_2_upper;
	person_2_lower_ = person_2_lower;
	looking_for_start_ = true;
	missing_person_buffer_ = 10;
	setup_done_ = true;
}
InstanceGrabber::InstanceGrabber(int objective_code,
								 const cv::Scalar& person_1_upper,
								 const cv::Scalar& person_1_lower,
								 const cv::Scalar& person_2_upper,
								 const cv::Scalar& person_2_lower) {
	Setup(objective_code,person_1_upper,person_1_lower,person_2_upper,person_2_lower);
}

void InstanceGrabber::Setup(int objective_code,
		 const cv::Scalar& person_1_upper,
		 const cv::Scalar& person_1_lower,
		 const cv::Scalar& person_2_upper,
		 const cv::Scalar& person_2_lower,
		 const cv::Scalar& bag) {
	assert(objective_code==R_TWO_PERSON_BAG_STEAL ||
		   objective_code==R_TWO_PERSON_BAG_EXCHANGE);
	objective_code_ = objective_code;
	person_1_upper_ = person_1_upper;
	person_1_lower_ = person_1_lower;
	person_2_upper_ = person_2_upper;
	person_2_lower_ = person_2_lower;
	bag_ = bag;
	looking_for_start_ = true;
	missing_person_buffer_ = 10;
	setup_done_ = true;
	already_stolen_ = false;
}
InstanceGrabber::InstanceGrabber(int objective_code,
							 const cv::Scalar& person_1_upper,
							 const cv::Scalar& person_1_lower,
							 const cv::Scalar& person_2_upper,
							 const cv::Scalar& person_2_lower,
							 const cv::Scalar& bag) {
	Setup(objective_code,person_1_upper,person_1_lower,person_2_upper,person_2_lower,bag);
}

void InstanceGrabber::Setup(int objective_code,
							const cv::Scalar& person_1_upper,
							const cv::Scalar& person_1_lower) {
	assert(objective_code==R_ONE_PERSON_TRACK);
	objective_code_ = objective_code;
	person_1_upper_ = person_1_upper;
	person_1_lower_ = person_1_lower;

	looking_for_start_ = true;
	missing_person_buffer_ = 5;
	setup_done_ = true;
}

void InstanceGrabber::Setup(int objective_code,
								 const cv::Scalar& person_1_upper,
								 const cv::Scalar& person_1_lower,
								 const cv::Scalar& bag) {
	assert(objective_code==R_ONE_PERSON_BAG_UNATTENDED);
	objective_code_ = objective_code;
	person_1_upper_ = person_1_upper;
	person_1_lower_ = person_1_lower;
	bag_ = bag;
	looking_for_start_ = true;
	missing_person_buffer_ = 10;
	setup_done_ = true;
}

void InstanceGrabber::Update(int frame_number,const PersonList& people,const BagList& bags) {
	if(!setup_done_) return;
	frame_number_ = frame_number;
	switch(objective_code_) {
	case R_ONE_PERSON_TRACK:
		UpdateOnePersonTrack(people);
		break;
	case R_ONE_PERSON_BAG_UNATTENDED:
		UpdateOnePersonBagUnattended(people,bags);
		break;
	case R_TWO_PERSON_BAG_EXCHANGE:
		UpdateTwoPersonBagExchange(people,bags);
		break;
	case R_TWO_PERSON_BAG_STEAL:
		UpdateTwoPersonBagSteal(people,bags);
		break;
	case R_TWO_PERSON_MEET:
		UpdateTwoPersonMeet(people);
		break;
	}
}
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
					std::cout<<"Starting instance"<<std::endl;
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
							std::cout<<"starting instance"<<std::endl;
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
				std::cout<<"starting instance"<<std::endl;
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
					std::cout<<"starting instance"<<std::endl;

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
				std::cout<<"starting instance"<<std::endl;
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

void InstanceGrabber::PrintInstances(std::ostream& out) {
	//TODO: if the instance isn't over then use now as the END_TIME
	if(!setup_done_) return;
	for(uint instance_index=0;instance_index<instances_.size();instance_index++) {
		out<<std::endl<<std::endl;
		out<<"Start: "<<instances_[instance_index][START_TIME];
		out<<" | End: "<<instances_[instance_index][END_TIME];
	}
}

bool InstanceGrabber::EventInProgress() const {
	 return !looking_for_start_;
}

bool InstanceGrabber::AlreadyStolen() const {
	return already_stolen_;
}
