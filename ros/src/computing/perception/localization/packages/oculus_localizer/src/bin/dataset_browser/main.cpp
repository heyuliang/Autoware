/*
 * main.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */


#include <iostream>

#include <QApplication>
#include "DatasetBrowser.h"


int main (int argc, char *argv[])
{
	QApplication mainApp(argc, argv);

	DatasetBrowser dbs;
	dbs.show();

	return mainApp.exec();
}
