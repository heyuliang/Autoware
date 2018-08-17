/*
 * MeidaiBag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#include <exception>
#include <algorithm>
#include <string>

#include <nmea_msgs/Sentence.h>
#include <gnss/geo_pos_conv.hpp>
#include "datasets/MeidaiBag.h"


using namespace std;
using namespace Eigen;

string MeidaiBag::dSetName = "Nagoya University";

struct GnssLocalizerState
{
	double roll_=0, pitch_=0, yaw_=0;
	double orientation_time_=0, position_time_=0;
	double latitude=0, longitude=0, height=0;
	ros::Time current_time_=ros::Time(0), orientation_stamp_=ros::Time(0);
	geo_pos_conv geo, last_geo;
};


MeidaiBag::MeidaiBag(const string &path)
{
	bagfd = new rosbag::Bag(path);
	cameraRawBag = new RandomAccessBag(*bagfd, "/camera1/image_raw");
}


MeidaiBag::~MeidaiBag()
{
	delete(cameraRawBag);
	delete(bagfd);
}


size_t
MeidaiBag::size() const
{
	return cameraRawBag->size();
}


CameraPinholeParams
MeidaiBag::getCameraParameter()
{
	throw runtime_error("Not implemented");
}


cv::Mat
MeidaiBag::getMask()
{
	return cv::Mat();
}


MeidaiDataItem&
MeidaiBag::at(dataItemId i) const
{
	// XXX: Stub
	throw runtime_error("Not implemented");
}


void
Trajectory::push_back(const PoseTimestamp &pt)
{
	if (size()>0)
		assert(pt.timestamp > back().timestamp);
	return Parent::push_back(pt);
}


uint32_t
Trajectory::find_lower_bound(const ros::Time &t) const
{
	auto it = std::lower_bound(begin(), end(), t,
		[](const PoseTimestamp &el, const ros::Time& tv)
			-> bool {return el.timestamp < tv;}
	);
	return it-begin();
}


PoseTimestamp
Trajectory::at(const ros::Time& t) const
{
	return Parent::at(find_lower_bound(t));
}


PoseTimestamp
Trajectory::interpolate (const ros::Time& t) const
{
	assert (t < (*end()).timestamp);

	uint32_t i0 = find_lower_bound(t),
		i1 = i0+1;
	return PoseTimestamp::interpolate(Parent::at(i0), Parent::at(i1), t);
}


std::vector<std::string> splitSentence(const std::string &string)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, ','))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}


void convertNMEASentenceToState (nmea_msgs::SentencePtr &msg, GnssLocalizerState &state)
{
	vector<string> nmea = splitSentence(msg->sentence);
	try {

		if (nmea.at(0).compare(0, 2, "QQ") == 0)
	    {
			state.orientation_time_ = stod(nmea.at(3));
			state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
			state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
			state.yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
			state.orientation_stamp_ = msg->header.stamp;
	    }

	    else if (nmea.at(0) == "$PASHR")
	    {
			state.orientation_time_ = stod(nmea.at(1));
			state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
			state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
			state.yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
	    }

	    else if(nmea.at(0).compare(3, 3, "GGA") == 0)
	    {
			state.position_time_ = stod(nmea.at(1));
			state.latitude = stod(nmea.at(2));
			state.longitude = stod(nmea.at(4));
			state.height = stod(nmea.at(9));
			state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
	    }

	    else if(nmea.at(0) == "$GPRMC")
	    {
			state.position_time_ = stoi(nmea.at(1));
			state.latitude = stod(nmea.at(3));
			state.longitude = stod(nmea.at(5));
			state.height = 0.0;
			state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
	    }

	} catch(const exception &e) {}
}


PoseTimestamp createFromState(const GnssLocalizerState &state)
{
	TQuaternion q(state.roll_, state.pitch_, state.yaw_);
	Vector3d p(state.geo.x(), state.geo.y(), state.geo.z());
	Pose pt = Pose::from_Pos_Quat(p, q);
	return pt;
}


void createTrajectoryFromGnssBag (RandomAccessBag &bagsrc, Trajectory &trajectory, int plane_number)
{
	const double orientationTimeout = 10.0;

	if (bagsrc.getTopic() != "/nmea_sentence")
		throw runtime_error("Not GNSS bag");

//	geo_pos_conv geoconv, last_geo;
	GnssLocalizerState state;
	state.geo.set_plane(plane_number);

	trajectory.clear();

	for (uint32_t ix=0; ix<bagsrc.size(); ix++) {

		auto currentMessage = bagsrc.at<nmea_msgs::Sentence>(ix);
		ros::Time current_time = currentMessage->header.stamp;
		convertNMEASentenceToState(currentMessage, state);

		if (fabs(state.orientation_stamp_.toSec() - currentMessage->header.stamp.toSec()) > orientationTimeout) {
			double dt = sqrt(pow(state.geo.x() - state.last_geo.x(), 2) + pow(state.geo.y() - state.last_geo.y(), 2));
			const double threshold = 0.2;
			if (dt > threshold) {
				// create fake orientation
				state.yaw_ = atan2(state.geo.x() - state.last_geo.x(), state.geo.y() - state.last_geo.y());
				state.roll_ = 0;
				state.pitch_ = 0;
				PoseTimestamp px = createFromState(state);
				px.timestamp = current_time;
				trajectory.push_back(px);
				state.last_geo = state.geo;
				continue;
			}
		}

		double e = 1e-2;
		if (fabs(state.orientation_time_ - state.position_time_) < e) {
			PoseTimestamp px = createFromState(state);
			px.timestamp = current_time;
			trajectory.push_back(px);
		}
	}
}


PoseTimestamp
PoseTimestamp::interpolate(
	const PoseTimestamp &p1,
	const PoseTimestamp &p2,
	const ros::Time &t)
{
	assert (p1.timestamp>=t and t<=p2.timestamp);
	double r = (t - p1.timestamp).toSec() / (p2.timestamp - p1.timestamp).toSec();
	return Pose::interpolate(p1, p2, r);
}
