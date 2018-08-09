/*
 * DatasetBrowser.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#include <iostream>

#include "DatasetBrowser.h"


using namespace std;


DatasetBrowser::DatasetBrowser(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);
}

DatasetBrowser::~DatasetBrowser()
{}


void
DatasetBrowser::on_timelineSlider_sliderMoved(int v)
{
	cout << v << endl;
}
