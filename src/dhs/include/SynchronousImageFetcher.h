/*
 * SynchronousImageFetcher.h
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

#ifndef SYNCHRONOUSIMAGEFETCHER_H_
#define SYNCHRONOUSIMAGEFETCHER_H_
#include <string>
#include "common.h"
#include "ImageFetcher.h"


class SynchronousImageFetcher {
	public:
	DISALLOW_COPY_AND_ASSIGN(SynchronousImageFetcher);
	SynchronousImageFetcher(const std::string& first_topic_name,const std::string& second_topic_name);
	SynchronousImageFetcher(const std::string& first_topic_name,const std::string& second_topic_name,const std::string& third_topic_name);

	int GetFrame(cv::Mat& first_frame,cv::Mat& second_frame);
	int GetFrame(cv::Mat& first_frame,cv::Mat& second_frame,cv::Mat& third_frame);

	private:
	ImageFetcher first_handle,second_handle,third_handle;
	const bool third_topic_exists_;
};

#endif /* SYNCHRONOUSIMAGEFETCHER_H_ */
