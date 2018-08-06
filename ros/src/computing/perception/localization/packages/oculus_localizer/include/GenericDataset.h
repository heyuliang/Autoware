/*
 * GenericDataset.h
 *
 *  Created on: Aug 3, 2018
 *      Author: sujiwo
 */

#ifndef _GENERICDATASET_H_
#define _GENERICDATASET_H_


#include <cstdint>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "VMap.h"


typedef uint64_t dataItemId;
typedef uint64_t timestamp_t;


class GenericDataset;


class GenericDataItem
{
public:

	virtual ~GenericDataItem();

	virtual cv::Mat getImage() = 0;

	virtual Eigen::Vector3d getPosition() const = 0;

	virtual Eigen::Quaterniond getOrientation() const = 0;

	virtual uint64_t getId() const = 0;

};


class GenericDataset
{
public:

	virtual ~GenericDataset()
	{}

	virtual size_t size() const = 0;

	virtual CameraPinholeParams getCameraParameter() = 0;

	virtual cv::Mat getMask() = 0;

	virtual GenericDataItem& at(const int i) const = 0;

	void dump(const std::string &filename="");
};




#endif /* _GENERICDATASET_H_ */
