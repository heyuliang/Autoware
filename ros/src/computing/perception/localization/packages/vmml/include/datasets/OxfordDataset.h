/*
 * OxfordDataset.h
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */

#ifndef _OXFORDDATASET_H_
#define _OXFORDDATASET_H_

#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <array>
#include <set>
#include <opencv2/opencv.hpp>

#include "VMap.h"
#include "utilities.h"
#include "GenericDataset.h"



/*
 * XXX: Oxford Timestamp is in Microsecond
 */


enum GroundTruthSrc {
	GPS,
	INS
};


/*
 * These values are constants to shift X/Y coordinates to more `reasonable' values
 */
const double
	OriginCorrectionEasting = -620248.53,
	OriginCorrectionNorthing = -5734882.47;


struct GpsPose
{
	uint64_t timestamp;
	double
		easting,
		northing,
		altitude,
		latitude,
		longitude;
};


struct InsPose : public GpsPose
{
	double
		roll,
		pitch,
		yaw;
};


class OxfordDataset;
struct OxfordDataItem : public GenericDataItem
{
	friend class OxfordDataset;
	dataItemId iId;
	timestamp_t timestamp;
	Pose groundTruth;
	OxfordDataset *parent;

	OxfordDataItem():
		parent(NULL)
	{}

	OxfordDataItem(OxfordDataset *p):
		parent(p)
	{}

	enum StereoImageT {
		StereoLeft,
		StereoCenter,
		StereoRight
	};
	cv::Mat getImage (StereoImageT which) ;
	cv::Mat getImage ()
	{ return getImage(StereoImageT::StereoCenter); }

	inline Eigen::Vector3d getPosition() const
	{ return groundTruth.position(); }

	Eigen::Quaterniond getOrientation() const
	{ return groundTruth.orientation(); }

	dataItemId getId() const
	{ return iId; }

	inline virtual timestamp_t getTimestamp()
	{ return timestamp; }

private:
	std::string getPath(StereoImageT t=StereoCenter) const;
};


class OxfordDataset: public GenericDataset
{
public:

	OxfordDataset (const OxfordDataset &cp);

	OxfordDataset (const std::string &dirpath, const std::string &modelDir, GroundTruthSrc gts=GroundTruthSrc::INS);
	virtual ~OxfordDataset();

	inline size_t size() const
	{ return stereoTimestamps.size(); }

	CameraPinholeParams getCameraParameter()
	{ return oxfCamera; }

	void dumpGroundTruth(const std::string &fp=std::string());

	OxfordDataItem &at(dataItemId i) const;

	inline OxfordDataItem& atTime (timestamp_t t) const
	{ return const_cast<OxfordDataItem&>(stereoRecords.at(t)); }

	friend struct OxfordDataItem;
	cv::Mat undistort (cv::Mat &src);

	cv::Mat getMask();

	OxfordDataset timeSubset (double startTimeOffsetSecond=0, double mappingDurationSecond=-1) const;

	inline std::string getName() const
	{ return dSetName; }

	// Bug: Do Not set ratio more than once
	void setZoomRatio (float r);


protected:
	CameraPinholeParams oxfCamera;

	std::string oxfPath;

	std::vector<timestamp_t> stereoTimestamps;

	std::map<timestamp_t,OxfordDataItem> stereoRecords;

	std::vector<GpsPose> gpsPoseTable;

	std::vector<InsPose> insPoseTable;

	cv::Mat distortionLUT_center_x, distortionLUT_center_y;

	float zoomRatio=1.0;

	static std::string dSetName;

private:
	void loadIns ();
	void loadGps ();
	void loadTimestamps ();

	void createStereoGroundTruths();

	void loadModel (const std::string &modelDir);
};

#endif /* _OXFORDDATASET_H_ */
