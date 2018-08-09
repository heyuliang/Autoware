/*
 * dataset_browser.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: sujiwo
 */

#include <iostream>

#include <QApplication>
#include <QWidget>

#include "ui_dataset_browser.h"
#include "GenericDataset.h"
#include "OxfordDataset.h"


using namespace std;


class DatasetBrowserFrm : public QWidget
{
//Q_OBJECT

private slots:
	void on_timelineSlider_sliderMoved(int value)
	{
		cout << value << endl;
	}
};


int main(int argc, char *argv[])
{
	QApplication datasetBrowserApp(argc, argv);

	DatasetBrowserFrm window;
	Ui::DatasetBrowser_frm dbfrm;
	dbfrm.setupUi(&window);

	window.show();

	return datasetBrowserApp.exec();
}
