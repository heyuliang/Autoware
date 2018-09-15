/*
 * CustomDataset.h
 *
 *  Created on: Aug 4, 2018
 *      Author: sujiwo
 */

#ifndef CUSTOMDATASET_H_
#define CUSTOMDATASET_H_

#include <string>
#include <vector>
#include "GenericDataset.h"
#include "VMap.h"


class CustomDataset;
class CustomDataItem : public GenericDataItem
{
public:
	friend class CustomDataset;

	inline CustomDataItem():
		id(0),
		imagePath(""),
		position(Eigen::Vector3d::Zero()),
		orientation(Eigen::Quaterniond::Identity())
	{}

	inline ~CustomDataItem() {}

	cv::Mat getImage() const;

	Eigen::Vector3d getPosition() const
	{ return position; }

	Eigen::Quaterniond getOrientation() const
	{ return orientation; }

	dataItemId getId() const
	{ return id; }

	ptime getTimestamp() const
	{ throw std::runtime_error("Not implemented"); }

	typedef std::shared_ptr<CustomDataItem> Ptr;
	typedef std::shared_ptr<CustomDataItem const> ConstPtr;

protected:
	uint64_t id;
	std::string imagePath;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};


class CustomDataset : public GenericDataset
{
public:

	CustomDataset(const std::string &dataDirPath);
	virtual ~CustomDataset();

	size_t size() const;

	CameraPinholeParams getCameraParameter();

	cv::Mat getMask()
	{ return mask; }

	CustomDataItem& at(dataItemId i) const;

	GenericDataItem::ConstPtr get(dataItemId i) const;

	void setZoomRatio (float r)
	{}

	float getZoomRatio () const
	{ return 1.0; }


protected:
	const std::string rootPath;

	std::vector<CustomDataItem> records;

	CameraPinholeParams cparams;

	cv::Mat mask;
};

#endif /* CUSTOMDATASET_H_ */
