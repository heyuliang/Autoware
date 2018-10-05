/*
 * FileProperty.h
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#ifndef _RVIZ_PLUGINS_FILEPROPERTY_H_
#define _RVIZ_PLUGINS_FILEPROPERTY_H_

#include <QWidget>
#include <rviz/properties/property.h>

class FileProperty: public rviz::Property
{
public:
	FileProperty(	const QString& name = QString(),
					const QString& description = QString(),
					Property* parent = 0,
					const char *changed_slot = 0,
					QObject* receiver = 0 );

	virtual QWidget* createEditor( QWidget* parent,
					const QStyleOptionViewItem& option );

public Q_SLOTS:
	virtual void onButtonClick();

private:


};

#endif /* _RVIZ_PLUGINS_FILEPROPERTY_H_ */
