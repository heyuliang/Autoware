/*
 * MapPoint.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef MAPPOINT_H_
#define MAPPOINT_H_

#include <Eigen/Core>
#include <vector>
#include <tuple>
#include <opencv2/core.hpp>
#include "VMap.h"


class KeyFrame;


struct KeyMapPoint {
	KeyFrame *keyframe;
	int keypointIdx;
};


namespace boost {
namespace serialization {
	template <class Archive>
		void serialize (Archive & ar, MapPoint &mappoint, const unsigned int version);
}
}


class MapPoint
{
public:

	MapPoint();
	MapPoint(const Eigen::Vector3d &p);
	virtual ~MapPoint();

	void createDescriptor(const std::vector<KeyMapPoint> &visibleIn);

	cv::Mat getDescriptor() const
	{ return descriptor.clone(); }

	double X() const
	{ return position.x(); }

	double Y() const
	{ return position.y(); }

	double Z() const
	{ return position.z(); }

	Eigen::Vector3d getPosition () const
	{ return position; }

	inline void setPosition (const Eigen::Vector3d &pw)
	{ position = pw; }

	mpid getId () const
	{ return id; }

protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, MapPoint &mappoint, const unsigned int version);


private:

	mpid id;

	Eigen::Vector3d position;

	// Best Descriptor
	cv::Mat descriptor;

	static mpid nextId;
};

#endif /* MAPPOINT_H_ */
