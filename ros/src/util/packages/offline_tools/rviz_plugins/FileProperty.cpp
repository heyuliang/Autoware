/*
 * FileProperty.cpp
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#include <QFileDialog>
#include <rviz/properties/line_edit_with_button.h>
#include "FileProperty.h"


FileProperty::FileProperty(
	const QString& name,
	const QString& description,
	Property* parent,
	const char *changed_slot,
	QObject* receiver) :

	rviz::Property(name, QVariant(), description, parent, changed_slot, receiver)
{}


QWidget*
FileProperty::createEditor( QWidget* parent, const QStyleOptionViewItem& option )
{
	rviz::LineEditWithButton *wEdit = new rviz::LineEditWithButton(this);
	wEdit->setFrame(false);
	QPushButton *b = wEdit->button();
//	connect(b, SIGNAL())
	return wEdit;
}
