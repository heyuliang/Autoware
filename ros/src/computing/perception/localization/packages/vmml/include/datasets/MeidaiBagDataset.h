/*
 * MeidaiBag.h
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <string>
#include <memory>
#include <vector>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>

#include "utilities.h"
#include "datasets/GenericDataset.h"
#include "datasets/RandomAccessBag.h"


#ifndef _MEIDAIBAG_H_
#define _MEIDAIBAG_H_


struct PoseTimestamp : public Pose
{
	PoseTimestamp():
		Pose()
	{ timestamp = ros::Time(0); }

	PoseTimestamp(const Pose &p)
	{
		m_matrix = p.matrix();
		timestamp = ros::Time(0);
	}

	static PoseTimestamp interpolate(
		const PoseTimestamp &p1,
		const PoseTimestamp &p2,
		const ros::Time &t);

	ros::Time timestamp;

	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		// Eigen matrix
		ar << boost::serialization::base_object<Pose>(*this);

		// Ros timestamp
		const decltype(timestamp.sec) tbuf[2] =
			{ timestamp.sec, timestamp.nsec };
		ar << tbuf;
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		// Eigen matrix
		ar >> boost::serialization::base_object<Pose>(*this);

		// Ros timestamp
		decltype(timestamp.sec) tbuf[2];
		ar >> tbuf;
		timestamp.sec = tbuf[0], timestamp.nsec = tbuf[1];
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};


class Trajectory : public std::vector<PoseTimestamp>
{
public:

	friend class boost::serialization::access;

	void push_back(const PoseTimestamp &);

	// Return nearest element of provided time
	PoseTimestamp at(const ros::Time&) const;

	// Interpolate value
	PoseTimestamp interpolate (const ros::Time&) const;

private:
	uint32_t
	find_lower_bound(const ros::Time&) const;

	typedef std::vector<PoseTimestamp> Parent;

	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{ ar & boost::serialization::base_object<Parent>(*this);}

};


class MeidaiDataItem : public GenericDataItem
{
};


class MeidaiBagDataset : public GenericDataset
{
public:

	MeidaiBagDataset(const std::string &filePath, double startTimeOffsetSecond=0, double mappingDurationSecond=-1, const std::string &calibrationPath=std::string());
	virtual ~MeidaiBagDataset();

	size_t size() const;

	CameraPinholeParams getCameraParameter();

	cv::Mat getMask();

	MeidaiDataItem& at(dataItemId i) const;

	const Trajectory& getGnssTrajectory() const
	{ return gnssTrack; }

protected:
	static std::string dSetName;
	rosbag::Bag *bagfd;
	RandomAccessBag *cameraRawBag;

	const boost::filesystem::path bagPath;

private:
	void loadCache ();
	void doLoadCache (const std::string &);
	void createCache ();
	void writeCache (const std::string&);

	Trajectory gnssTrack;
	Trajectory ndtTrack;
};


void createTrajectoryFromGnssBag (RandomAccessBag &bagsrc, Trajectory &trajectory, int plane_number=7);
void createTrajectoryFromNDT (RandomAccessBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack);

#endif /* _MEIDAIBAG_H_ */
