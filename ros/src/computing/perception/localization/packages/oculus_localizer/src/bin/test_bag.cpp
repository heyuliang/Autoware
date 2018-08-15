/*
 * test_bag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <rosbag/structures.h>

#include <sensor_msgs/Image.h>


using namespace std;


template<typename Tag, typename Tag::type M>
struct Rob {
  friend typename Tag::type get(Tag) {
    return M;
  }
};

struct MessageInstance_P1 {
  typedef rosbag::ConnectionInfo const* rosbag::MessageInstance::*type;
  friend type get(MessageInstance_P1);
};

struct MessageInstance_P2 {
  typedef rosbag::IndexEntry const rosbag::MessageInstance::*type;
  friend type get(MessageInstance_P2);
};

struct MessageInstance_P3 {
  typedef rosbag::Bag const* rosbag::MessageInstance::*type;
  friend type get(MessageInstance_P3);
};

template struct Rob<MessageInstance_P2, &rosbag::MessageInstance::index_entry_>;



class RandomAccessBag : public rosbag::View
{
public:

RandomAccessBag(
	rosbag::Bag const& bag,
	boost::function<bool(rosbag::ConnectionInfo const*)> query,
	ros::Time const& start_time = ros::TIME_MIN,
	ros::Time const& end_time = ros::TIME_MAX,
	bool const& reduce_overlap = false) :
		rosbag::View::View(bag, query, start_time, end_time, reduce_overlap)
{
}

template<typename T>
boost::shared_ptr<T>
operator[](int p)
{
}

protected:

void createCache ()
{
	iterator it = begin();
	msgPtr.reserve(size());

	for (uint32_t i=0; i<size(); i++) {
		rosbag::MessageInstance m = *it;
//		msgPtr.at(i) = rosbag::MessageInstance(m.connection_info_, m.index_entry_, *m.bag_);
		it++;
	}
}

private:
std::vector<rosbag::MessageInstance> msgPtr;

};



int main (int argc, char *argv[])
{
	rosbag::Bag bagstore;
	bagstore.open("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");

	vector<string> topics = {
		"/camera1/image_raw",
		"/nmea_sentence",
		"/velodyne_packets"
	};
	rosbag::View bagview(bagstore, rosbag::TopicQuery(topics[0]));
	uint32_t sz = bagview.size();

	auto it = bagview.begin();
	auto &m = *it;
	vector<const rosbag::ConnectionInfo*> lConn = bagview.getConnections();

//	for (auto &m: bagview) {
	for (auto it=bagview.begin(); it!=bagview.end(); ++it) {
		auto &m = *it;
		string mTopic = m.getTopic();
		rosbag::IndexEntry const ie = m.*get(MessageInstance_P2());
		continue;
	}

	bagstore.close();
	cout << "Done" << endl;
	return 0;
}
