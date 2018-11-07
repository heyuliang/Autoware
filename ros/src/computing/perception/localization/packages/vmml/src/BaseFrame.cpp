/*
 * BaseFrame.cpp
 *
 *  Created on: Oct 31, 2018
 *      Author: sujiwo
 */

#include "BaseFrame.h"


using namespace Eigen;


typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


BaseFrame::BaseFrame()
{
	// TODO Auto-generated constructor stub
}

BaseFrame::~BaseFrame()
{
	// TODO Auto-generated destructor stub
}


Eigen::Vector2d
BaseFrame::project (const Eigen::Vector3d &pt3) const
{
	Vector3d ptx = projectionMatrix() * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
}


Vector3d
BaseFrame::transform (const Eigen::Vector3d &pt3) const
{
	Vector4d P = externalParamMatrix4() * pt3.homogeneous();
	return P.hnormalized();
}


poseMatrix4
BaseFrame::externalParamMatrix4 () const
{
	return createExternalParamMatrix4(mPose);
}


Eigen::Matrix4d
BaseFrame::createExternalParamMatrix4(const Pose &ps)
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = ps.orientation().toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*ps.position());
	return ex;
}


Eigen::Matrix<double,3,4>
BaseFrame::projectionMatrix () const
{
	assert (cameraParam != nullptr);
	return cameraParam->toMatrix() * externalParamMatrix4();
}


Vector3d
BaseFrame::normal() const
{
	return externalParamMatrix4().block(0,0,3,3).transpose().col(2);
}


void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, const cv::Mat &mask)
{
	assert (image.empty() == false);

	fd->detectAndCompute(
		image,
		mask,
		fKeypoints,
		fDescriptors,
		false);
}
