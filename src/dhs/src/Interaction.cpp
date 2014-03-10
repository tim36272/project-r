/*
 * Interaction.cpp
    Copyright (C) 2014 Timothy Sweet

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

#include "dhs/Interaction.h"

namespace interaction {
void checkForInteractions(const BlobPtr& first_blob, const std::vector<BlobPtr>& other_blobs, Interactions& interactions) {
	if(!first_blob->bag()) {
		/*
		 * Single Person
		 */
		InteractionStatePtr single_person(new InteractionState);
		if(checkForSinglePerson(first_blob,single_person)) {
			//This will either insert, or not if it already exists
			std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(single_person->getHash(),single_person));
			if(!inserted.second)
				inserted.first->second->add_seen_at(first_blob->lastSeen());
		}
		/*
		 * Two blob events need a second blob
		 */
		std::vector<BlobPtr>::const_iterator second_blob_it = other_blobs.begin();
		while(second_blob_it!=other_blobs.end()) {
			if(*second_blob_it == first_blob) {
				++second_blob_it;
				continue;
			}
			/*
			 * Two Person meeting
			 */
			InteractionStatePtr two_person_meeting(new InteractionState);
			if(checkForTwoPeopleMeeting(first_blob,*second_blob_it,two_person_meeting)) {
				//This will either insert, or not if it already exists
				std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(two_person_meeting->getHash(),two_person_meeting));
				if(!inserted.second)
					inserted.first->second->add_seen_at(first_blob->lastSeen());
			}
			//things with bags
			if(first_blob->bag()) {
				const BlobDescriptorDecoratedKBPtr owner = other_blobs.at(first_blob->owner());
				/*
				 * Bag interaction
				 */
				InteractionStatePtr bag_abandoned(new InteractionState);
				if(checkForBagAbandoned(owner,first_blob,bag_abandoned)) {
					//This will either insert, or not if it already exists
					std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(bag_abandoned->getHash(),bag_abandoned));
					if(!inserted.second)
						inserted.first->second->add_seen_at(first_blob->lastSeen());
				}
				/*
				 * Bag Exchange
				 */
				InteractionStatePtr bag_exchange(new InteractionState);
				if(checkForBagExchange(owner,first_blob,*second_blob_it,bag_exchange)) {
					//This will either insert, or not if it already exists
					std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(bag_exchange->getHash(),bag_exchange));
					if(!inserted.second)
						inserted.first->second->add_seen_at(first_blob->lastSeen());
				}
				/*
				 * Bag Steal
				 */
				InteractionStatePtr bag_steal(new InteractionState);
				if(checkForBagExchange(owner,first_blob,*second_blob_it,bag_steal)) {
					//This will either insert, or not if it already exists
					std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(bag_steal->getHash(),bag_steal));
					if(!inserted.second)
						inserted.first->second->add_seen_at(first_blob->lastSeen());
				}
			}
			++second_blob_it;
		}
	}
	else { //it's a bag

	}
}

bool checkForSinglePerson(const BlobDescriptorDecoratedKBPtr blob, interaction::InteractionStatePtr interaction) {
	if(blob->bag()) return false;

	interaction->set_agent_1_id(blob->Id());
	interaction->set_agent_2_id(interaction::kNotSet);
	interaction->set_bag_id(interaction::kNotSet);
	interaction->add_seen_at(blob->lastSeen());
	interaction->set_interaction(InteractionState::single_person);
	//this event is always relevant
	return true;
}

bool checkForTwoPeopleMeeting(const BlobDescriptorDecoratedKBPtr first_blob, const BlobDescriptorDecoratedKBPtr second_blob, InteractionStatePtr interaction) {
	//must both be visible at the same time
	if(first_blob->lastSeen() != second_blob->lastSeen()) return false;
	if(first_blob->bag() || second_blob->bag()) return false;

	//TODO: add depth checking
	//check if they are close to eachother
	int separation =  utility::separation(first_blob->getLastFilteredBound(),second_blob->getLastFilteredBound());
	//max_separation is the sum of the bounds' widths
	int max_separation = first_blob->getLastFilteredBound().x/4 + second_blob->getLastFilteredBound().x/4;

	if(separation < max_separation) {
		interaction->set_agent_1_id(first_blob->Id());
		interaction->set_agent_2_id(second_blob->Id());
		interaction->set_bag_id(interaction::kNotSet);
		interaction->add_seen_at(first_blob->lastSeen());
		interaction->set_interaction(InteractionState::two_person_meeting);
		return true;
	}
	return false;
}
bool checkForBagAbandoned(BlobDescriptorDecoratedKBPtr owner, const BlobDescriptorDecoratedKBPtr bag, InteractionStatePtr interaction) {
	//check if they are close to each other
	int separation =  utility::separation(bag->getLastFilteredBound(),owner->getLastFilteredBound());
	int max_separation = bag->getLastFilteredBound().x + owner->getLastFilteredBound().x;

	if(separation > max_separation) {
		interaction->set_agent_1_id(owner->Id());
		interaction->set_agent_2_id(interaction::kNotSet);
		interaction->set_bag_id(bag->Id());
		interaction->add_seen_at(bag->lastSeen());
		interaction->set_interaction(InteractionState::bag_abandon);
		return true;
	}
	return false;
}
bool checkForBagExchange(BlobDescriptorDecoratedKBPtr owner, const BlobDescriptorDecoratedKBPtr bag, const BlobDescriptorDecoratedKBPtr receiver, InteractionStatePtr interaction) {
	//must all be visible
	//allow 1 for update asynchronacy
	int bag_last_seen = bag->lastSeen() - 1;
	if(owner->lastSeen() < bag_last_seen || receiver->lastSeen() < bag_last_seen) return false;

	//check if they are bag and owner are close
	int bag_owner_separation =  utility::separation(bag->getLastFilteredBound(),owner->getLastFilteredBound());
	int max_bag_owner_separation = bag->getLastFilteredBound().x + owner->getLastFilteredBound().x;

	//check if they are bag and receiver are close
	int bag_receiver_separation =  utility::separation(bag->getLastFilteredBound(),receiver->getLastFilteredBound());
	int max_bag_receiver_separation = bag->getLastFilteredBound().x + receiver->getLastFilteredBound().x;

	if( (bag_owner_separation < max_bag_owner_separation) && (bag_receiver_separation < max_bag_receiver_separation)) {
		interaction->set_agent_1_id(owner->Id());
		interaction->set_agent_2_id(owner->Id());
		interaction->set_bag_id(bag->Id());
		interaction->add_seen_at(bag->lastSeen());
		interaction->set_interaction(InteractionState::bag_exchange);
		return true;
	}
	return false;
}

bool checkForBagSteal(BlobDescriptorDecoratedKBPtr owner, const BlobDescriptorDecoratedKBPtr bag, const BlobDescriptorDecoratedKBPtr receiver, InteractionStatePtr interaction) {
	//receiver must be visible
	//allow 1 for update asynchronacy
	int bag_last_seen = bag->lastSeen() - 1;
	if(receiver->lastSeen() < bag_last_seen) return false;

	//check if they are bag and owner are close
	int bag_owner_separation =  utility::separation(bag->getLastFilteredBound(),owner->getLastFilteredBound());
	int max_bag_owner_separation = bag->getLastFilteredBound().x + owner->getLastFilteredBound().x;

	//check if they are bag and receiver are close
	int bag_receiver_separation =  utility::separation(bag->getLastFilteredBound(),receiver->getLastFilteredBound());
	int max_bag_receiver_separation = bag->getLastFilteredBound().x + receiver->getLastFilteredBound().x;

	if( (bag_owner_separation >
	max_bag_owner_separation) && (bag_receiver_separation < max_bag_receiver_separation)) {
		interaction->set_agent_1_id(owner->Id());
		interaction->set_agent_2_id(owner->Id());
		interaction->set_bag_id(bag->Id());
		interaction->add_seen_at(bag->lastSeen());
		interaction->set_interaction(InteractionState::bag_steal);
		return true;
	}
	return false;
}

void print(const Interactions& interactions) {
	std::cout<<"Blob history: ";
	interaction::Interactions::const_iterator interaction_it = interactions.begin();
	while(interaction_it != interactions.end()) {
		std::cout<<"Blobs: "
				 <<interaction_it->second->agent_1_id_<<","
				 <<interaction_it->second->agent_2_id_
				 <<" are having interaction type "
				 <<interaction_it->second->interaction_<<": ";
		std::vector<std::pair<int,int> >::const_iterator event_it = interaction_it->second->instances_.begin();
		while(event_it!=interaction_it->second->instances_.end()) {
			std::cout<<"("<<event_it->first<<","<<event_it->second<<")";
			++event_it;
		}
		std::cout<<std::endl;
		++interaction_it;
	}
	std::cout<<std::endl;
}

void cull(Interactions& interactions) {
	int last_event=0;
	interaction::Interactions::const_iterator interaction_it = interactions.begin();
	while(interaction_it != interactions.end()) {
		std::vector<std::pair<int,int> >::const_iterator event_it = interaction_it->second->instances_.begin();
		while(event_it!=interaction_it->second->instances_.end()) {
			if(event_it->second > last_event) last_event = event_it->second;
			event_it++;
		}
		++interaction_it;
	}

	while(interaction_it != interactions.end()) {
		std::vector<std::pair<int,int> >::iterator event_it = interaction_it->second->instances_.begin();
		while(event_it!=interaction_it->second->instances_.end()) {
			if( (event_it->second - event_it->first < 10) && (event_it->second < last_event-10) ) {
				std::vector<std::pair<int,int> >::iterator to_be_erased = event_it;
				event_it++;
				interaction_it->second->instances_.erase(to_be_erased);
			} else {
				event_it++;
			}
		}
		++interaction_it;
	}
}

}; // namespace interaction
