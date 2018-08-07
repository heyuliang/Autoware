/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <sstream>
#include <cstdio>
#include <string>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "utilities.h"


using namespace std;
using namespace Eigen;


string
str(const tf::Vector3 &v)
{
#define SEP ','

	stringstream ss;
	ss << v.x()
		<< SEP
		<< v.y()
		<< SEP
		<< v.z();
	return ss.str();
}


string
str(const tf::Quaternion &q)
{
	stringstream ss;
	ss << q.x()
		<< SEP
		<< q.y()
		<< SEP
		<< q.z()
		<< SEP
		<< q.w();
	return ss.str();
}



int main (int argc, char *argv[])
{
	float
		roll = stod(argv[1]),
		pitch = stod(argv[2]),
		yaw = stod(argv[3]);

	Quaterniond q = fromRPY(roll, pitch, yaw);
	printf ("x=%f, y=%f, z=%f, w=%f\n", q.x(), q.y(), q.z(), q.w());

//	TQuaternion q(0.019, 0.013, 0.625, 0.781);
//	cout << q.toRotationMatrix() << endl;
//	Vector3d v = quaternionToRPY(q);
//	printf ("Roll=%f, Pitch=%f, Yaw=%f\n", v[0], v[1], v[2]);

//	float x=-253.335, y=956.179, z=112.158;
//	float qx=-0.556, qy=-0.437, qz=0.443, qw=0.551;
//
//	tf::Transform World_Baselink;
//	World_Baselink.setOrigin(tf::Vector3(-251.705, 955.814, 111.010));
//	World_Baselink.setRotation(tf::Quaternion(0.019, 0.013, 0.625, 0.780));
//
//	tf::Transform Baselink_CameraView;
//	Baselink_CameraView.setOrigin(tf::Vector3(-0.000, 1.720, 1.070));
//	Baselink_CameraView.setRotation(tf::Quaternion(-0.723, 0.007, 0.002, 0.691));
//
//	tf::Transform World_CameraView = World_Baselink * Baselink_CameraView;
//
//	Affine3d mq;
//	poseTFToEigen(World_CameraView, mq);
//	Matrix4d m1 = mq.matrix();
//	cout << m1 << endl;
//
//	Vector3d ve;
//	Quaterniond qe;
//	vectorTFToEigen(World_CameraView.getOrigin(), ve);
//	quaternionTFToEigen(World_CameraView.getRotation(), qe);
//	TTransform m2 = TTransform::from_Pos_Quat(ve, qe);
//	cout << m2.matrix() << endl;

	return 0;
}
