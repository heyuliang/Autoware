/*
 * DatasetBrowser.h
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#ifndef _DATASETBROWSER_H_
#define _DATASETBROWSER_H_

#include <QWidget>
#include "ui_dataset_browser.h"


class DatasetBrowser: public QWidget
{
Q_OBJECT

public:

	explicit DatasetBrowser(QWidget *parent=NULL);
	virtual ~DatasetBrowser();

private slots:
	void on_timelineSlider_sliderMoved(int value);

private:
	Ui::DatasetBrowser_frm ui;
};

#endif /* _DATASETBROWSER_H_ */
