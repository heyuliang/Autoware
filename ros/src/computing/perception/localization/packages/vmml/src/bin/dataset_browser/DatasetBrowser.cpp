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
#include <SequenceSLAM.h>
#include <QImage>
#include <QFileDialog>

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
	saveImageButton = ui.saveImageButton;
}

DatasetBrowser::~DatasetBrowser()
{}


void
DatasetBrowser::on_timelineSlider_sliderMoved(int v)
{
	return setImageOnPosition(v);
}


void
DatasetBrowser::on_saveImageButton_clicked(bool checked)
{
	QString fname = QFileDialog::getSaveFileName(this, tr("Save Image"));
	cv::Mat image = openDs->get(timelineSlider->value())->getImage();
	cv::imwrite(fname.toStdString(), image);
}


void
DatasetBrowser::changeDataset(GenericDataset *ds)
{
	openDs = ds;
	timelineSlider->setRange(0, ds->size()-1);
	dataItem0 = ds->get(0);
	setImageOnPosition(0);
}


void
DatasetBrowser::setImageOnPosition (int v)
{
	if (v<0 or v>=openDs->size())
		throw runtime_error("Invalid time position");

	auto curItem = openDs->get(v);
	cv::Mat image = curItem->getImage();

	QImage curImage = fromCvMat(image);
	frame->setImage(curImage);

	auto ts = curItem->getTimestamp() - dataItem0->getTimestamp();
	double tsd = double(ts.total_microseconds())/1e6;

	stringstream ss;
	ss << fixed << setprecision(2) << tsd;
	timeOffsetLabel->setText(QString::fromStdString(ss.str()));
}
