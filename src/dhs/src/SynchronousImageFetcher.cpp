/*
 * SynchronousImageFetcher.cpp
    Copyright (C) 2013  Timothy Sweet

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
#include "dhs/SynchronousImageFetcher.h"
SynchronousImageFetcher::SynchronousImageFetcher(const std::string& first_topic_name,
											     const std::string& second_topic_name):
	first_handle(first_topic_name),
	second_handle(second_topic_name),
	third_topic_exists_(false) {}
SynchronousImageFetcher::SynchronousImageFetcher(const std::string& first_topic_name,
												 const std::string& second_topic_name,
												 const std::string& third_topic_name):
	first_handle(first_topic_name),
	second_handle(second_topic_name),
	third_handle(third_topic_name),
	third_topic_exists_(true) {}

int SynchronousImageFetcher::GetFrame(cv::Mat& first_frame,cv::Mat& second_frame) {
	assert(!third_topic_exists_);
	//check if both depth and rgb handles have updated information to provide
	//TODO:synchronize these with sequence numbers
	if(first_handle.IsUpdated() && second_handle.IsUpdated()) {
		//get the frames
		int first_sequence_number = first_handle.GetFrame(first_frame);
		assert(first_sequence_number!=FRAME_NOT_UPDATED);
		int second_sequence_number = second_handle.GetFrame(second_frame);
		assert(second_sequence_number!=FRAME_NOT_UPDATED);
		return first_sequence_number;
	}
	return FRAME_NOT_UPDATED;
}

int SynchronousImageFetcher::GetFrame(cv::Mat& first_frame,cv::Mat& second_frame, cv::Mat& third_frame) {
	assert(third_topic_exists_);
	//check if both depth and rgb handles have updated information to provide
	//TODO:synchronize these with sequence numbers
	if(first_handle.IsUpdated() && second_handle.IsUpdated() && third_handle.IsUpdated()) {
		//get the frames
		int first_sequence_number = first_handle.GetFrame(first_frame);
		assert(first_sequence_number!=FRAME_NOT_UPDATED);
		int second_sequence_number = second_handle.GetFrame(second_frame);
		assert(second_sequence_number!=FRAME_NOT_UPDATED);
		int third_sequence_number = third_handle.GetFrame(third_frame);
		assert(third_sequence_number!=FRAME_NOT_UPDATED);
		return first_sequence_number;
	}
	return FRAME_NOT_UPDATED;
}
