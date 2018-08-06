/*
 * utilities.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include "utilities.h"


using namespace std;
using namespace Eigen;


Quaterniond fromRPY (double roll, double pitch, double yaw)
{
	roll /= 2.0;
	pitch /= 2.0;
	yaw /= 2.0;
	double
		ci = cos(roll),
		si = sin(roll),
		cj = cos(pitch),
		sj = sin(pitch),
		ck = cos(yaw),
		sk = sin(yaw),
		cc = ci*ck,
		cs = ci*sk,
		sc = si*ck,
		ss = si*sk;
	Quaterniond q;
	q.x() = cj*sc - sj*cs;
	q.y() = cj*ss + sj*cc;
	q.z() = cj*cs - sj*sc;
	q.w() = cj*cc + sj*ss;
	q.normalize();
	return q;
}


Vector3d quaternionToRPY (const Quaterniond &q)
{
	Eigen::Matrix3d m = q.toRotationMatrix();

	double cy = sqrt(m(0,0)*m(0,0) + m(1,0)*m(1,0));
	double ax, ay, az;
	if (cy > std::numeric_limits<double>::epsilon()*4.0) {
		ax = atan2( m(2, 1),  m(2, 2));
		ay = atan2(-m(2, 0),  cy);
		az = atan2( m(1, 0),  m(0,0));
	}
	else {
        ax = atan2(-m(1, 2),  m(1, 1));
        ay = atan2(-m(2, 0),  cy);
        az = 0.0;
	}
	return Vector3d(ax, ay, az);
}


TTransform
TTransform::from_XYZ_RPY (
	const Eigen::Vector3d &pos,
	double roll, double pitch, double yaw)
{
	Quaterniond q = fromRPY(roll, pitch, yaw);
	return TTransform::from_Pos_Quat(pos, q);
}


TTransform
TTransform::from_Pos_Quat(const Vector3d &pos, const Quaterniond &orient)
{
	Affine3d t;
	t = Eigen::Translation3d(pos) * orient;
	return t;
}


#include <sstream>

string
TTransform::str() const
{
	stringstream ss;
	ss << "x=" << position().x()
		<< " y=" << position().y()
		<< " z=" << position().z()
		<< ", qx=" << orientation().x()
		<< " qy=" << orientation().y()
		<< " qz=" << orientation().z()
		<< " qw=" << orientation().w();
	return ss.str();
}


Vector4d vectorFromQuaternion (const Quaterniond &q)
{
	return Vector4d(q.x(), q.y(), q.z(), q.w());
}


void
TTransform::displacement (
	const TTransform &other,
	double &linear, double &angular) const
{
	linear = (this->position()-other.position()).norm();

	Vector4d q1 = vectorFromQuaternion(this->orientation().normalized()),
		q2 = vectorFromQuaternion(other.orientation().normalized());
	angular = acos(
		q1.head(3).dot(q2.head(3)) /
		(q1.head(3).norm() * q2.head(3).norm())
	);
}



