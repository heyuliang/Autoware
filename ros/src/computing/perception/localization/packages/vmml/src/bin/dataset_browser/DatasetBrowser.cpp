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

#include <opencv2/core.hpp>
#include <SequenceSLAM.h>
#include <QImage>
#include <QFileDialog>

#include "DatasetBrowser.h"


using namespace std;


DatasetBrowser::DatasetBrowser(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);

	timelineSlider = ui.timelineSlider;
	frame = ui.frame;
	timeOffsetLabel = ui.timeOffsetLabel;
	saveImageButton = ui.saveImageButton;
	playButton = ui.playButton;
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
	if (fname.length()==0)
		return;
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

	auto ts = curItem->getTimestamp() - dataItem0->getTimestamp();
	double tsd = double(ts.total_microseconds())/1e6;

	stringstream ss;
	ss << fixed << setprecision(2) << tsd;
	timeOffsetLabel->setText(QString::fromStdString(ss.str()));

	cv::Mat image = curItem->getImage();
	cv::cvtColor(image, image, CV_BGR2RGB);
	QImage curImage (image.data, image.cols, image.rows, image.step[0], QImage::Format_RGB888);
	frame->setImage(curImage);
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
		timelineSlider->setDisabled(true);
		for (int p=startPos; p<=timelineSlider->maximum(); p++) {
			timelineSlider->setSliderPosition(p);
			setImageOnPosition(p);
			if (playStarted == false)
				break;
			if(p < timelineSlider->maximum()) {
				ptime t1 = openDs->get(p)->getTimestamp();
				ptime t2 = openDs->get(p+1)->getTimestamp();
				tduration td = t2-t1;
				std::this_thread::sleep_for(std::chrono::milliseconds(td.total_milliseconds()));
			}
		}
		timelineSlider->setDisabled(false);
	};

	if (checked==true) {
		cout << "Play\n";
		playStarted = true;
		playerThread = new std::thread(playThreadFn);
	}

	else {
		cout << "Stop\n";
		playStarted = false;
		playerThread->join();
		delete(playerThread);
	}

	return;
}
