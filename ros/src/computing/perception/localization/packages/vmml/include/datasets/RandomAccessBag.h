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

	RandomAccessBag(
		rosbag::Bag const &bag, const std::string &topic,
		double startTimeOffsetSecond=0, double mappingDurationSecond=-1);

	template<typename T>
	boost::shared_ptr<T>
	at (int p)
	{
		assert(p>=0 and p<size());
		return instantiate<T>(msgPtr.at(p));
	}

	~RandomAccessBag();

	std::string getTopic ()
	{ return conn->topic; }

//	template<typename T>
//	bool
//	readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data) const
//	{
//		ros::M_string::const_iterator i = call_private::checkField(bagstore, fields, field_name, sizeof(T), sizeof(T), required);
//		if (i == fields.end())
//			return false;
//		memcpy(data, i->second.data(), sizeof(T));
//		return true;
//	}
//
//	template<class T>
//	boost::shared_ptr<T>
//	instantiate (const rosbag::IndexEntry &index_entry) const
//	{
//		call_private::decompressChunk(bagstore, index_entry.chunk_pos);
//
//		// Read the message header
//		ros::Header header;
//		uint32_t data_size;
//		uint32_t bytes_read;
//		call_private::readMessageDataHeaderFromBuffer(bagstore, *(access_private::current_buffer_(bagstore)), index_entry.offset, header, data_size, bytes_read);
//
//		// Read the connection id from the header
//		uint32_t connection_id;
//		readField(*header.getValues(), rosbag::CONNECTION_FIELD_NAME, true, &connection_id);
//
//		boost::shared_ptr<T> p = boost::make_shared<T>();
//
//		ros::serialization::PreDeserializeParams<T> predes_params;
//		predes_params.message = p;
//		predes_params.connection_header = conn->header;
//		ros::serialization::PreDeserialize<T>::notify(predes_params);
//
//		// Deserialize the message
//		ros::serialization::IStream s(access_private::current_buffer_(bagstore)->getData() + index_entry.offset + bytes_read, data_size);
//		ros::serialization::deserialize(s, *p);
//
//		return p;
//	}


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
