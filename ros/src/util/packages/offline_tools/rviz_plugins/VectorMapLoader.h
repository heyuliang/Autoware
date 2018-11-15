/*
 * VectorMapLoader.h
 *
 *  Created on: Nov 14, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPLOADER_H_
#define _VECTORMAPLOADER_H_


#include <string>
#include <boost/filesystem.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <vector_map/vector_map.h>


class VectorMapLoader : public vector_map::VectorMap
{
public:
	friend class VectorMapDisplay;

	VectorMapLoader(const std::string &directory);

	inline visualization_msgs::MarkerArray::ConstPtr ConstPtr()
	{ return visualization_msgs::MarkerArray::ConstPtr(&marker_array); }

private:
	boost::filesystem::path vmDir;
	visualization_msgs::MarkerArray marker_array;

	void loadAll ();
};


#endif /* _VECTORMAPLOADER_H_ */
