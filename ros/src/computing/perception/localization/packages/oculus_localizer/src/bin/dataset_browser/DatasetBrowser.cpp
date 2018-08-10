/*
 * DatasetBrowser.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/core.hpp>
#include <QImage>

#include "DatasetBrowser.h"


using namespace std;


QImage fromCvMat (cv::Mat &matsrc)
{
	QImage img(matsrc.data, matsrc.cols, matsrc.rows, matsrc.step[0], QImage::Format_RGB888);
	return img;
}


DatasetBrowser::DatasetBrowser(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);

	timelineSlider = ui.timelineSlider;
	frame = ui.frame;
	timeOffsetLabel = ui.timeOffsetLabel;
}

DatasetBrowser::~DatasetBrowser()
{}


void
DatasetBrowser::on_timelineSlider_sliderMoved(int v)
{
	return setImageOnPosition(v);
}


void
DatasetBrowser::changeDataset(GenericDataset *ds)
{
	openDs = ds;
	timelineSlider->setRange(0, ds->size()-1);
	setImageOnPosition(0);

}


void
DatasetBrowser::setImageOnPosition (int v)
{
	if (v<0 and v>=openDs->size())
		throw runtime_error("Invalid time position");

	cv::Mat image = openDs->at(v).getImage();
	QImage curImage = fromCvMat(image);
	frame->setImage(curImage);

	timestamp_t ts = openDs->at(v).getTimestamp() - openDs->at(0).getTimestamp();
	double tsd = double(ts)/1e6;

	stringstream ss;
	ss << fixed << setprecision(2) << tsd;
	timeOffsetLabel->setText(QString::fromStdString(ss.str()));
}
