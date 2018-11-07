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
	void on_playButton_clicked(bool checked);

private:
	Ui::DatasetBrowser_frm ui;

	GenericDataset *openDs;

	QSlider *timelineSlider;
	RatioLayoutedFrame *frame;
	QLabel *timeOffsetLabel;
	QPushButton *saveImageButton;

	QPushButton *playButton;

private:
	void setImageOnPosition (int v);
	void startPlayBag();
	void stopPlayBag();
	void disableControlsOnPlaying (bool state);

	GenericDataItem::ConstPtr dataItem0;
};

#endif /* _DATASETBROWSER_H_ */
