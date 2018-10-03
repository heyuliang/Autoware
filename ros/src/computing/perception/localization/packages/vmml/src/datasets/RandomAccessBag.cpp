/*
 * RandomAccessBag.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#include <algorithm>
#include <rosbag/query.h>
#include "datasets/RandomAccessBag.h"




/*
 * Only allows one single topic
 */
RandomAccessBag::RandomAccessBag
	(rosbag::Bag const &bag, const std::string &topic,
	const ros::Time &startTime, const ros::Time &stopTime):

	rosbag::View::View(),
	bagstore(const_cast<rosbag::Bag&>(bag))

{
	addQuery(bag, rosbag::TopicQuery(topic), startTime, stopTime);

	conn = getConnections()[0];
	createCache();
}


RandomAccessBag::~RandomAccessBag()
{
}


void
RandomAccessBag::createCache()
{
	rosbag::View::size();
	iterator it = begin();
	size_t sz = this->size();
	msgPtr.resize(sz);

	for (uint32_t p=0; p<sz; p++) {
		rosbag::MessageInstance &m = *it;
		rosbag::IndexEntry const ie = access_private::index_entry_(m);
		msgPtr.at(p) = ie;
		++it;
	}
}


uint32_t
RandomAccessBag::getPositionAtDurationSecond (const double S) const
{
	ros::Duration Sd (S);
	ros::Time Tx = msgPtr.at(0).time + Sd;
	assert (Tx>= msgPtr.at(0).time and Tx<=msgPtr.back().time);

	auto it = std::lower_bound(msgPtr.begin(), msgPtr.end(), Tx,
		[](const rosbag::IndexEntry &iptr, const ros::Time &t)
		{ return (iptr.time < t); }
	);
	return it - msgPtr.begin();
}


//ros::Duration
//RandomAccessBag::length() const
//{
//	return msgPtr.back().time - msgPtr.at(0).time;
//}


