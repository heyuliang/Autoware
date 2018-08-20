/*
 * main.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */


#include <iostream>

#include <QApplication>
#include "DatasetBrowser.h"
#include "datasets/OxfordDataset.h"


int main (int argc, char *argv[])
{
	QApplication mainApp(argc, argv);

	OxfordDataset oxf("/home/sujiwo/Data/Oxford/2014-12-02-15-30-08", "/home/sujiwo/Sources/robotcar-dataset-sdk/models");
	oxf.setZoomRatio(0.5);

	DatasetBrowser dbs;
	dbs.changeDataset(&oxf);
	dbs.show();

	return mainApp.exec();
}
