/*
 * DatasetBrowser.h
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#ifndef _DATASETBROWSER_H_
#define _DATASETBROWSER_H_

#include <QWidget>
#include <QSlider>
#include <QLabel>

#include "ui_dataset_browser.h"
#include "ratio_layouted_frame.h"

#include "GenericDataset.h"


class DatasetBrowser: public QWidget
{
Q_OBJECT

public:

	explicit DatasetBrowser(QWidget *parent=NULL);
	virtual ~DatasetBrowser();

	void changeDataset(GenericDataset *ds);

public slots:
	void on_timelineSlider_sliderMoved(int value);

private:
	Ui::DatasetBrowser_frm ui;

	GenericDataset *openDs;

	QSlider *timelineSlider;
	RatioLayoutedFrame *frame;
	QLabel *timeOffsetLabel;

private:
	void setImageOnPosition (int v);

};

#endif /* _DATASETBROWSER_H_ */
