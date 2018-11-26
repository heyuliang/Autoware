/*
 * DatasetBrowser.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <SequenceSLAM.h>
#include <QImage>
#include <QFileDialog>

#include "DatasetBrowser.h"
#include "BaseFrame.h"

#include "datasets/MeidaiBagDataset.h"


using namespace std;

using Path = boost::filesystem::path;


// XXX: Find a way to specify these values from external input
CameraPinholeParams meidaiCamera1Params(
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920,			// width
	1440			// height
);



DatasetBrowser::DatasetBrowser(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);

	timelineSlider = ui.timelineSlider;
	frame = ui.frame;
	timeOffsetLabel = ui.timeOffsetLabel;
	saveImageButton = ui.saveImageButton;
	playButton = ui.playButton;
	enableLidarScanRender = ui.enableLidarScanRender;
}

DatasetBrowser::~DatasetBrowser()
{}


void
DatasetBrowser::on_timelineSlider_sliderMoved(int v)
{
	return setImageOnPosition(v);
}


/*
void
DatasetBrowser::on_saveImageButton_clicked(bool checked)
{
	QString fname = QFileDialog::getSaveFileName(this, tr("Save Image"));
	if (fname.length()==0)
		return;
	cv::Mat image = openDs->get(timelineSlider->value())->getImage();
	cv::imwrite(fname.toStdString(), image);
}
*/


std::string dPoseLean (const Pose &frame)
{
	stringstream ss;
	ss << fixed << setprecision(6);
	auto P = frame.position();
	auto Q = frame.orientation();

	ss << P.x() << ' ' << P.y() << ' ' << P.z() << ' ';
	ss << Q.x() << ' ' << Q.y() << ' ' << Q.z() << ' ' << Q.w();

	return ss.str();
}

const string defaultImageExtension = "png";

void DatasetBrowser::on_saveImageButton_clicked(bool checked)
{
	Path cwd = boost::filesystem::current_path();
	int currentId = timelineSlider->value();
	string imageName = to_string(currentId) + '.' + defaultImageExtension;
	Path fullName = cwd / Path(imageName);

	auto cDataItem = openDs->get(currentId);
	cv::Mat image = cDataItem->getImage();
	cv::imwrite(fullName.string(), image);

	// write camera coordinate to STDOUT
	Pose fp = cDataItem->getPose();
	cout << cDataItem->getId() << ' ' << toSeconds(cDataItem->getTimestamp()) << ' ' << dPoseLean(fp) <<  endl << flush;
}


// XXX: Change this
const string lidarCalibrationParams("/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/params/64e-S2.yaml");

void
DatasetBrowser::changeDataset(GenericDataset::Ptr ds, datasetType ty)
{
	openDs = ds;
	timelineSlider->setRange(0, ds->size()-1);
	dataItem0 = ds->get(0);

	if (ty==DatasetBrowser::MeidaiType) {
		MeidaiBagDataset* meidaiDs = static_cast<MeidaiBagDataset*>(ds);
		meidaiDs->setLidarParameters(lidarCalibrationParams, string(), defaultLidarToCameraTransform);
		meidaiPointClouds = meidaiDs->getLidarScanBag();
	}

	setImageOnPosition(0);
}


bool isTimeInside (const LidarScanBag::Ptr &bg, ros::Time Tx)
{
	return (Tx>=bg->startTime() and Tx<bg->stopTime());
}


void
DatasetBrowser::setImageOnPosition (int v)
{
	if (v<0 or v>=openDs->size())
		throw runtime_error("Invalid time position");

	auto curItem = openDs->get(v);

	auto ts = curItem->getTimestamp() - dataItem0->getTimestamp();
	double tsd = double(ts.total_microseconds())/1e6;

	stringstream ss;
	ss << fixed << setprecision(2) << tsd;
	timeOffsetLabel->setText(QString::fromStdString(ss.str()));

	cv::Mat image = curItem->getImage();
	cv::cvtColor(image, image, CV_BGR2RGB);

	try {
		if (enableLidarScanRender->isChecked()) {

			auto imageTime = ros::Time::fromBoost(curItem->getTimestamp());
			if (meidaiPointClouds!=nullptr and isTimeInside(meidaiPointClouds, imageTime)) {

				uint32_t pcIdx = meidaiPointClouds->getPositionAtTime(imageTime);
				auto pointCloud = meidaiPointClouds->at(pcIdx);
				auto projections = projectScan(pointCloud);

				drawPoints(image, projections);
			}
		}
	} catch (const std::exception &e) {}

	QImage curImage (image.data, image.cols, image.rows, image.step[0], QImage::Format_RGB888);
	frame->setImage(curImage);
}


void
DatasetBrowser::disableControlsOnPlaying (bool state)
{
	timelineSlider->setDisabled(state);
	saveImageButton->setDisabled(state);
}


void
DatasetBrowser::on_playButton_clicked(bool checked)
{
	static bool playStarted = false;
	static std::thread *playerThread = NULL;

	std::function<void()> playThreadFn =
	[&]()
	{
		const int startPos = timelineSlider->sliderPosition();
		disableControlsOnPlaying(true);
		for (int p=startPos; p<=timelineSlider->maximum(); p++) {

			ptime t1x = getCurrentTime();
			timelineSlider->setSliderPosition(p);
			setImageOnPosition(p);
			if (playStarted == false)
				break;

			if(p < timelineSlider->maximum()) {
				ptime t1 = openDs->get(p)->getTimestamp();
				ptime t2 = openDs->get(p+1)->getTimestamp();
				ptime t2x = getCurrentTime();
				tduration tdx = t2x - t1x;	// processing overhead
				tduration td = (t2-t1) - tdx;
				std::this_thread::sleep_for(std::chrono::milliseconds(td.total_milliseconds()));
			}
		}
		disableControlsOnPlaying(false);
	};

	if (checked==true) {
		playStarted = true;
		playerThread = new std::thread(playThreadFn);
	}

	else {
		playStarted = false;
		playerThread->join();
		delete(playerThread);
	}

	return;
}


std::vector<cv::Point2f>
DatasetBrowser::projectScan
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan)
const
{
	std::vector<cv::Point2f> projections;

	// Create fake frame
	BaseFrame frame;
	frame.setPose(defaultLidarToCameraTransform);
	frame.setCameraParam(&meidaiCamera1Params);

	projections.resize(lidarScan->size());
	int i=0;
	for (auto it=lidarScan->begin(); it!=lidarScan->end(); ++it) {
		auto &pts = *it;
		Vector3d pt3d (pts.x, pts.y, pts.z);

		auto p3cam = frame.externalParamMatrix4() * pt3d.homogeneous();
		if (p3cam.z() >= 0) {
			auto p2d = frame.project(pt3d);
			projections[i] = cv::Point2f(p2d.x(), p2d.y());
			++i;
		}
	}

	return projections;
}


const cv::Scalar projectionColor (255,0,0);

void
DatasetBrowser::drawPoints
(cv::Mat &target, const std::vector<cv::Point2f> &pointList)
{
	for (auto &pt2d: pointList) {
		if ((pt2d.x>=0 and pt2d.x<target.cols) and (pt2d.y>=0 and pt2d.y<target.rows)) {
			cv::circle(target, pt2d, 2, projectionColor, -1);
		}
	}
}
