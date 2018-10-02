/*
 * MeidaiLocalizerGNSS.cpp
 *
 *  Created on: Aug 17, 2018
 *      Author: sujiwo
 */

#include <datasets/MeidaiBagDataset.h>
#include <iostream>
#include <string>
#include <vector>
#include <limits>

#include <nmea_msgs/Sentence.h>
#include <gnss/geo_pos_conv.hpp>


using namespace std;


const Vector3d
	GNSS_Translation_Offset (18500, 93800, -33);


class wrong_nmea_sentence : public exception
{};

class invalid_gnss_position : public exception
{};


/*
These are expected results (without offset)

header:
  seq: 198
  stamp:
    secs: 1482726415
    nsecs: 842756032
  frame_id: "map"
pose:
  position:
    x: -18266.4001572
    y: -93879.5007049
    z: 43.0621
  orientation:
    x: -0.0742241380626
    y: 0.029065332244
    z: -0.962810799329
    w: -0.258149856645
*/

const double positionInvalid = std::numeric_limits<double>::max();


struct GnssLocalizerState
{
	double roll_=0, pitch_=0, yaw_=0;
	double orientation_time_=0, position_time_=0;
	double latitude=positionInvalid, longitude=positionInvalid, height=positionInvalid;
	ros::Time current_time_=ros::Time(0), orientation_stamp_=ros::Time(0);
	geo_pos_conv geo, last_geo;
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
		try {
			state.position_time_ = stod(nmea.at(1));
			state.latitude = stod(nmea.at(2));
			state.longitude = stod(nmea.at(4));
			state.height = stod(nmea.at(9));
			state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
		} catch (std::invalid_argument &e) {
			throw wrong_nmea_sentence();
		}
	}

	else if(nmea.at(0) == "$GPRMC")
	{
		state.position_time_ = stoi(nmea.at(1));
		state.latitude = stod(nmea.at(3));
		state.longitude = stod(nmea.at(5));
		state.height = 0.0;
		state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
	}

	else
		throw wrong_nmea_sentence();
}


PoseTimestamp createFromState(const GnssLocalizerState &state)
{
	if (state.latitude==positionInvalid or state.longitude==positionInvalid)
		throw invalid_gnss_position();

	TQuaternion q(state.roll_, state.pitch_, state.yaw_);
	Vector3d p(state.geo.y(), state.geo.x(), state.geo.z());
	p = p + GNSS_Translation_Offset;
	Pose pt = Pose::from_Pos_Quat(p, q);
	return pt;
}


void createTrajectoryFromGnssBag (RandomAccessBag &bagsrc, Trajectory &trajectory, int plane_number)
{
	const double orientationTimeout = 10.0;

	if (bagsrc.getTopic() != "/nmea_sentence")
		throw runtime_error("Not GNSS bag");

	GnssLocalizerState state;
	state.geo.set_plane(plane_number);

	trajectory.clear();
	uint32_t nMsg = bagsrc.size();

	for (uint32_t ix=0; ix<nMsg; ix++) {
		cout << ix << "/" << bagsrc.size() << "         \r";

		auto currentMessage = bagsrc.at<nmea_msgs::Sentence>(ix);
		ros::Time current_time = currentMessage->header.stamp;

		try {
			convertNMEASentenceToState(currentMessage, state);
		} catch (const wrong_nmea_sentence &e)
		{
			continue;
		}

		if (fabs(state.orientation_stamp_.toSec() - currentMessage->header.stamp.toSec()) > orientationTimeout) {
			double dt = sqrt(pow(state.geo.x() - state.last_geo.x(), 2) + pow(state.geo.y() - state.last_geo.y(), 2));
			const double threshold = 0.2;
			if (dt > threshold) {
				// create fake orientation
				state.yaw_ = atan2(state.geo.x() - state.last_geo.x(), state.geo.y() - state.last_geo.y());
				state.roll_ = 0;
				state.pitch_ = 0;

				PoseTimestamp px;
				try {
					px = createFromState(state);
				} catch (invalid_gnss_position &e) {
					continue;
				}

				px.timestamp = current_time;
				trajectory.push_back(px);
				state.last_geo = state.geo;
				continue;
			}
		}

		double e = 1e-2;
		if (fabs(state.orientation_time_ - state.position_time_) < e) {

			PoseTimestamp px;
			try {
				px = createFromState(state);
			} catch (invalid_gnss_position &e) {
				continue;
			}

			px.timestamp = current_time;
			trajectory.push_back(px);
			continue;
		}
	}

	cout << "\nDone\n";
}


PoseTimestamp
PoseTimestamp::interpolate(
	const PoseTimestamp &p1,
	const PoseTimestamp &p2,
	const ros::Time &t)
{
	assert (p1.timestamp<=t and t<=p2.timestamp);
	double r = (t - p1.timestamp).toSec() / (p2.timestamp - p1.timestamp).toSec();
	return Pose::interpolate(p1, p2, r);
}
