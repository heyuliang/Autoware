/*
 * VectorMapDisplay.h
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPDISPLAY_H_
#define _VECTORMAPDISPLAY_H_

#include <string>
#include <boost/filesystem.hpp>
#include <rviz/display.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector_map/vector_map.h>


class VectorMapLoader : public vector_map::VectorMap
{
public:
	friend class VectorMapDisplay;

	VectorMapLoader(const std::string &directory);

private:
	boost::filesystem::path vmDir;
	visualization_msgs::MarkerArray marker_array;

	void loadAll ();
};



class VectorMapDisplay: public rviz::Display
{
Q_OBJECT

public:
	VectorMapDisplay();
	virtual ~VectorMapDisplay();

private
	Q_SLOTS:

protected:
	virtual void onInitialize();
};

#endif /* _VECTORMAPDISPLAY_H_ */
