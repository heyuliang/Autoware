/*
 * RandomAccessBag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _RANDOMACCESSBAG_H_
#define _RANDOMACCESSBAG_H_


#include <string>
#include <vector>
#include <utility>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>

#include "access_private.hpp"



ACCESS_PRIVATE_FIELD(rosbag::MessageInstance, rosbag::IndexEntry const, index_entry_);

//ACCESS_PRIVATE_FUN(rosbag::Bag,
//	void(uint64_t) const,
//	decompressChunk);
//
//ACCESS_PRIVATE_FIELD(rosbag::Bag, rosbag::Buffer*, current_buffer_);
//ACCESS_PRIVATE_FUN(rosbag::Bag,
//	void (rosbag::Buffer&, uint32_t, ros::Header&, uint32_t&, uint32_t&) const,
//	readMessageDataHeaderFromBuffer);
//
//ACCESS_PRIVATE_FUN(rosbag::Bag,
//	ros::M_string::const_iterator (ros::M_string const&, std::string const&, unsigned int, unsigned int, bool) const,
//	checkField);



class RandomAccessBag: public rosbag::View {
public:

	typedef std::shared_ptr<RandomAccessBag> Ptr;

	RandomAccessBag(
		rosbag::Bag const &bag, const std::string &topic,
		const ros::Time &startTime = ros::TIME_MIN,
		const ros::Time &endTime = ros::TIME_MAX);

	~RandomAccessBag();

	template<typename T>
	boost::shared_ptr<T>
	at (int position)
	{
		assert(position>=0 and position<size());
		return instantiate<T>(msgPtr.at(position));
	}

	RandomAccessBag subset(const ros::Time &start, ros::Duration &d) const;

	ros::Time timeAt (const int i) const
	{
		return msgPtr.at(i).time;
	}

	template<typename T>
	boost::shared_ptr<T>
	atDurationSecond (const double S)
	{
		return at<T> (getPositionAtDurationSecond(S));
	}

	std::string getTopic ()
	{ return conn->topic; }

	size_t size() const
	{ return static_cast<size_t>(size_cache_); }

	uint32_t getPositionAtDurationSecond (const double S) const;

	/*
	 * Duration of this view in ros::Duration
	 */
	 ros::Duration length() const
	 { return stopTime()-startTime(); }

	 ros::Time startTime() const
	 { return msgPtr.front().time; }

	 ros::Time stopTime() const
	 { return msgPtr.back().time; }

protected:
	void createCache();

	rosbag::Bag &bagstore;
	const rosbag::ConnectionInfo* conn;
	std::vector<rosbag::IndexEntry> msgPtr;

	template<class T>
	boost::shared_ptr<T>
	instantiate (const rosbag::IndexEntry &index_entry)
	{
		rosbag::MessageInstance *m = newMessageInstance(conn, index_entry, bagstore);
		return m->instantiate<T>();
	}


};

#endif /* _RANDOMACCESSBAG_H_ */
