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
#include <exception>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "utilities.h"
#include "CameraPinholeParams.h"


typedef uint64_t dataItemId;
typedef uint64_t timestamp_t;


class GenericDataset;


class GenericDataItem
{
public:

	typedef std::shared_ptr<GenericDataItem> Ptr;
	typedef std::shared_ptr<GenericDataItem const> ConstPtr;

	virtual ~GenericDataItem()
	{}

	virtual cv::Mat getImage() const = 0;

	virtual Pose getPose() const = 0;

	virtual Eigen::Vector3d getPosition() const = 0;

	virtual Eigen::Quaterniond getOrientation() const = 0;

	/*
	 * DataItem IDs are numbered from 0, thus they signify
	 * position in an array
	 */
	virtual dataItemId getId() const = 0;
//	{ return itemId; }

	virtual ptime getTimestamp() const = 0;
//	{ return iTimestamp; }

//protected:
//	dataItemId itemId;
//	ptime iTimestamp;

};


class GenericDataset
{
public:

	typedef std::shared_ptr<GenericDataset> Ptr;
	typedef std::shared_ptr<GenericDataset const> ConstPtr;

	GenericDataset()
	{}

	virtual ~GenericDataset()
	{}

	virtual size_t size() const = 0;

	virtual CameraPinholeParams getCameraParameter() const = 0;

	virtual cv::Mat getMask() = 0;

//	virtual const GenericDataItem& at(dataItemId i) const = 0;

	virtual GenericDataItem::ConstPtr get(dataItemId i) const = 0;

	virtual GenericDataItem::ConstPtr atDurationSecond (const double second) const = 0;

	void dump(const std::string &filename="");

	virtual void setZoomRatio (float r) = 0;

	virtual float getZoomRatio () const = 0;

	/*
	 * Duration of the dataset in seconds
	 */
	virtual double length() const;

protected:
	static std::string dSetName;
};




#endif /* _GENERICDATASET_H_ */
