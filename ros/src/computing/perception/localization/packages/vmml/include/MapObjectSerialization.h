/*
 * MapObjectSerialization.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sujiwo
 */

#ifndef MAPOBJECTSERIALIZATION_H_
#define MAPOBJECTSERIALIZATION_H_

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include "cvobj_serialization.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

#include "VMap.h"
#include "KeyFrame.h"
#include "MapPoint.h"


//BOOST_SERIALIZATION_SPLIT_FREE (KeyFrame);
//BOOST_SERIALIZATION_SPLIT_FREE (MapPoint);

BOOST_SERIALIZATION_SPLIT_FREE (Eigen::Vector3d);
BOOST_SERIALIZATION_SPLIT_FREE (Eigen::Quaterniond);
//typedef Eigen::Matrix<double,3,4> Mat_3_4;
//BOOST_SERIALIZATION_SPLIT_FREE (Mat_3_4);


namespace boost {
namespace serialization {


template <class Archive>
void save (
	Archive &ar,
	const Eigen::Vector3d &v3,
	const unsigned int version)
{
	double v[3] = {v3.x(), v3.y(), v3.z()};
	ar << v;
}

template <class Archive>
void load (
		Archive &ar,
		Eigen::Vector3d &v3,
		const unsigned int version)
{
	double v[3];
	ar >> v;
	v3.x() = v[0];
	v3.y() = v[1];
	v3.z() = v[2];
}


template <class Archive, typename _scalar>
void save (
	Archive &ar,
	const Eigen::Quaternion<_scalar> &q,
	const unsigned int version)
{
	double v[4] = {q.x(), q.y(), q.z(), q.w()};
	ar << v;
}


template <class Archive, typename _scalar>
void load (
		Archive &ar,
		Eigen::Quaternion<_scalar> &q,
		const unsigned int version)
{
	double v[4];
	ar >> v;
	q.x() = v[0];
	q.y() = v[1];
	q.z() = v[2];
	q.w() = v[3];
}


template <class Archive, typename T, int nrows, int ncols>
void save (
		Archive &ar,
		const Eigen::Matrix<T,nrows,ncols> &M,
		const unsigned int version)
{
	T mtable[nrows][ncols];
	for (int i=0; i<nrows; i++) {
		for (int j=0; j<ncols; j++) {
			mtable[i][j] = M(i,j);
		}
	}
	ar << mtable;
}


template <class Archive, typename T, int nrows, int ncols>
void load (
		Archive &ar,
		Eigen::Matrix<T,nrows,ncols> &M,
		const unsigned int version)
{
	T mtable[nrows][ncols];
	ar >> mtable;
	for (int i=0; i<nrows; i++) {
		for (int j=0; j<ncols; j++) {
			M(i,j) = mtable[i][j];
		}
	}
}


template <class Archive, typename T, int nrows, int ncols>
inline void serialize(
	Archive &ar,
	Eigen::Matrix<T,nrows,ncols> &M,
	const unsigned int version)
{
	split_free(ar, M, version);
}


template <class Archive>
void serialize (
	Archive &ar,
	KeyFrame &kf,
	const unsigned int version)
{
	ar &
		kf.id &
		kf.fKeypoints &
		kf.fDescriptors &
		kf.srcItemId;
	ar &
		kf.mPose &
		kf.cameraId;
	ar &
		kf.frCreationTime;
}


template <class Archive>
void serialize (
	Archive & ar,
	MapPoint &mp,
	const unsigned int version)
{
	ar &
		mp.id &
		mp.position &
		mp.descriptor;
}



}	/* serialization */
}	/* boost */

#endif /* MAPOBJECTSERIALIZATION_H_ */
