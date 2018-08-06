/*
 * CustomDataset.cpp
 *
 *  Created on: Aug 4, 2018
 *      Author: sujiwo
 */

#include <exception>
#include <opencv2/highgui.hpp>
#include "CustomDataset.h"


using namespace std;
using namespace Eigen;


cv::Mat
CustomDataItem::getImage()
{
	return cv::imread(this->imagePath, cv::IMREAD_GRAYSCALE);
}


CustomDataset::CustomDataset(const std::string &dataDirPath) :
	rootPath(dataDirPath)
{
	string groundTruthList = rootPath + "/pose.txt";
	ifstream inputfd (groundTruthList.c_str());
	if (!inputfd.is_open())
		throw std::runtime_error("Unable to open pose ground truth");

	string cameraParamsFile = rootPath + "/camera.txt";
	cparams = CameraPinholeParams::loadCameraParamsFromFile(cameraParamsFile);

	string line;
	while (true) {
		if (!getline(inputfd, line))
			break;

		CustomDataItem cItem;
		int iid;
		float timestamp;
		double x, y, z, qx, qy, qz, qw;
		sscanf (line.c_str(), "%d %f %lf %lf %lf %lf %lf %lf %lf", &iid, &timestamp,
			&x, &y, &z,
			&qx, &qy, &qz, &qw);
		cItem.id = iid;
		cItem.position << x, y, z;
		cItem.orientation.x() = qx;
		cItem.orientation.y() = qy;
		cItem.orientation.z() = qz;
		cItem.orientation.w() = qw;
		cItem.imagePath = rootPath + '/' + std::to_string(iid) + ".png";
		if (access(cItem.imagePath.c_str(), R_OK) != 0)
			throw std::runtime_error(string("No such file: ")+cItem.imagePath);

		records.push_back(cItem);
	}

	inputfd.close();
	mask = cv::imread(rootPath+"/mask.png", cv::IMREAD_GRAYSCALE);
}


CustomDataset::~CustomDataset() {}


size_t
CustomDataset::size() const
{
	return records.size();
}

CameraPinholeParams
CustomDataset::getCameraParameter()
{
	return cparams;
}


CustomDataItem&
CustomDataset::at(const int i) const
{
	return const_cast<CustomDataItem&>(records.at(i));
}
