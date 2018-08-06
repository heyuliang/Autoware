/*
 * triangulation.h
 *
 *  Created on: Jun 14, 2018
 *      Author: sujiwo
 */

#ifndef TRIANGULATION_H_
#define TRIANGULATION_H_

#include <Eigen/Eigen>


typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;


// `Optimal' triangulation
bool Triangulate(
	const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d &triangulated_point);


// Triangulates 2 posed views
bool TriangulateDLT(
	const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d &triangulated_point);

#endif /* TRIANGULATION_H_ */
