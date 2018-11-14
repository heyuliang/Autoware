/*
 * VectorMapDisplay.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#include <vector>
#include "VectorMapDisplay.h"


using namespace std;
using namespace boost::filesystem;


const vector<string> VectorMapFileNames
    {
      "idx.csv",
      "point.csv",
      "vector.csv",
      "line.csv",
      "area.csv",
      "pole.csv",
      "box.csv",
      "dtlane.csv",
      "node.csv",
      "lane.csv",
      "wayarea.csv",
      "roadedge.csv",
      "gutter.csv",
      "curb.csv",
      "whiteline.csv",
      "stopline.csv",
      "zebrazone.csv",
      "crosswalk.csv",
      "road_surface_mark.csv",
      "poledata.csv",
      "roadsign.csv",
      "signaldata.csv",
      "streetlight.csv",
      "utilitypole.csv",
      "guardrail.csv",
      "sidewalk.csv",
      "driveon_portion.csv",
      "intersection.csv",
      "sidestrip.csv",
      "curvemirror.csv",
      "wall.csv",
      "fence.csv",
      "railroad_crossing.csv"
    };



void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
	a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}


template <class T, class U>
U createObjectArray(const std::string& file_path)
{
	U obj_array;
	// NOTE: Autoware want to use map messages with or without /use_sim_time.
	// Therefore we don't set obj_array.header.stamp.
	// obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	obj_array.data = vector_map::parse<T>(file_path);
	return obj_array;
}


VectorMapLoader::VectorMapLoader(const std::string &directory) :
	vmDir(directory)
{
	if (!is_directory(vmDir))
		throw runtime_error("Not a directory");

	loadAll ();
}


void VectorMapLoader::loadAll()
{
	for (directory_entry &fe: directory_iterator(vmDir)) {

	}
}


VectorMapDisplay::VectorMapDisplay()
{
	// TODO Auto-generated constructor stub

}

VectorMapDisplay::~VectorMapDisplay()
{
	// TODO Auto-generated destructor stub
}

