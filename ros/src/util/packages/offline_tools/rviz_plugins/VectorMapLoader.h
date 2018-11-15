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
	typedef decltype(vector_map::Point::bx) ptScalar;
	friend class VectorMapDisplay;

	VectorMapLoader(const std::string &directory);

//	inline visualization_msgs::MarkerArray::ConstPtr ConstPtr()
//	{ return visualization_msgs::MarkerArray::ConstPtr(&marker_array); }

	visualization_msgs::MarkerArray::ConstPtr getAsMessages();

	void setPointOffset (const ptScalar x_offset, const ptScalar y_offset, const ptScalar z_offset);

private:

	boost::filesystem::path vmDir;

	decltype(vector_map::VectorMap::point_.map_) pointOrig;

//	visualization_msgs::MarkerArray marker_array;

	void loadAll ();
};


#endif /* _VECTORMAPLOADER_H_ */
