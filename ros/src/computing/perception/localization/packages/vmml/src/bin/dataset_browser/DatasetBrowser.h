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
#include <QPushButton>

#include "ui_dataset_browser.h"
#include "ratio_layouted_frame.h"

#include "datasets/GenericDataset.h"


class DatasetBrowser: public QWidget
{
Q_OBJECT

public:

	explicit DatasetBrowser(QWidget *parent=NULL);
	virtual ~DatasetBrowser();

	void changeDataset(GenericDataset *ds);

public slots:
	void on_timelineSlider_sliderMoved(int value);
	void on_saveImageButton_clicked(bool checked);

private:
	Ui::DatasetBrowser_frm ui;

	GenericDataset *openDs;

	QSlider *timelineSlider;
	RatioLayoutedFrame *frame;
	QLabel *timeOffsetLabel;
	QPushButton *saveImageButton;

private:
	void setImageOnPosition (int v);

};

#endif /* _DATASETBROWSER_H_ */
