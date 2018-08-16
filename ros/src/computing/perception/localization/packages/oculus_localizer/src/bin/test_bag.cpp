/*
 * test_bag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <vector>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <rosbag/structures.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <nmea_msgs/Sentence.h>
#include <gnss/geo_pos_conv.hpp>

#include <opencv2/highgui.hpp>

#include "datasets/RandomAccessBag.h"
#include "utilities.h"


using namespace std;
using Eigen::Vector3d;


struct GnssLocalizerState
{
	double roll_=0, pitch_=0, yaw_=0;
	double orientation_time_=0, position_time_=0;
	double latitude=0, longitude=0, height=0;
	ros::Time current_time_=ros::Time(0), orientation_stamp_=ros::Time(0);
};


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
	    }

	    else if(nmea.at(0) == "$GPRMC")
	    {
			state.position_time_ = stoi(nmea.at(1));
			state.latitude = stod(nmea.at(3));
			state.longitude = stod(nmea.at(5));
			state.height = 0.0;
	    }

	} catch(const exception &e) {}
}


Pose createFromState(const GnssLocalizerState &state, const geo_pos_conv &g)
{
	TQuaternion q(state.roll_, state.pitch_, state.yaw_);
	Vector3d p(g.x(), g.y(), g.z());
	return Pose::from_Pos_Quat(p, q);
}


void createTrackFromGnssBag (RandomAccessBag &bagsrc, vector<Pose> &trajectory, int plane_number=7)
{
	const double orientationTimeout = 10.0;

	if (bagsrc.getTopic() != "/nmea_sentence")
		throw runtime_error("Not GNSS bag");

	geo_pos_conv geoconv, last_geo;
	geoconv.set_plane(plane_number);
	GnssLocalizerState state;

	trajectory.clear();

	for (uint32_t ix=0; ix<bagsrc.size(); ix++) {
		auto currentMessage = bagsrc.at<nmea_msgs::Sentence>(ix);

		ros::Time current_time = currentMessage->header.stamp;
		convertNMEASentenceToState(currentMessage, state);

		if (fabs(state.orientation_stamp_.toSec() - currentMessage->header.stamp.toSec()) > orientationTimeout) {
			double dt = sqrt(pow(geoconv.x() - last_geo.x(), 2) + pow(geoconv.y() - last_geo.y(), 2));
			const double threshold = 0.2;
			if (dt > threshold) {
				// create fake orientation
				state.yaw_ = atan2(geoconv.x() - last_geo.x(), geoconv.y() - last_geo.y());
				state.roll_ = 0;
				state.pitch_ = 0;
				Pose px = createFromState(state, geoconv);
				trajectory.push_back(px);
				last_geo = geoconv;
				continue;
			}
		}

		double e = 1e-2;
		if (fabs(state.orientation_time_ - state.position_time_) < e) {
			Pose px = createFromState(state, geoconv);
			trajectory.push_back(px);
		}
	}
}


int main (int argc, char *argv[])
{
	rosbag::Bag bagstore;
	bagstore.open("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");

	RandomAccessBag ramImageBag (bagstore, "/camera1/image_raw");
	auto img0 = ramImageBag.at<sensor_msgs::Image>(0);

	RandomAccessBag ramGnssBag (bagstore, "/nmea_sentence");
	vector<Pose> track;
	createTrackFromGnssBag(ramGnssBag, track);

	return 0;
}
