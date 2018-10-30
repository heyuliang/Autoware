
/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// Auther : Hatem Darweesh
// Date   :  23/09/2018

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ostream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <tinyxml.h>
#include <proj_api.h>
#include "dirent.h"
#include "MapGrid.h"

#ifndef foreach
#define foreach BOOST_FOREACH
#endif

using namespace MAPPINGGRID_CPY_NS;

#define LINE_WIDTH 2.5
#define GROUND_RELATIVE_HRIGHT 8


int g_prev_sat_number = 0;
int g_max_sat = 0;
int g_min_sat = 100;
TiXmlElement* pHeadElem = 0;
long path_id = 0;

projPJ pj_latlong, pj_utm;

std::vector<GPS_Point> g_line_points;
MappingGrid g_GridMap;

void CreateTemplateDocument(TiXmlDocument& doc)
{
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "UTF-8", "" );
	TiXmlElement * mainElement = new TiXmlElement( "kml");
	mainElement->SetAttribute("xmlns", "http://www.opengis.net/kml/2.2");
	mainElement->SetAttribute("xmlns:gx", "http://www.google.com/kml/ext/2.2");
	mainElement->SetAttribute("xmlns:kml", "http://www.opengis.net/kml/2.2");
	mainElement->SetAttribute("xmlns:atom", "http://www.w3.org/2005/Atom");
	mainElement->LinkEndChild(new TiXmlElement("Document"));
	mainElement->FirstChild()->LinkEndChild(new TiXmlElement("name"));
	mainElement->FirstChild()->LastChild()->LinkEndChild(new TiXmlText("Extracted KML from PointCloud Data"));
	mainElement->FirstChild()->LinkEndChild(new TiXmlElement("open"));
	mainElement->FirstChild()->LastChild()->LinkEndChild(new TiXmlText("1"));

	doc.LinkEndChild( decl );
	doc.LinkEndChild( mainElement );
}

TiXmlElement* CreateStyleWithColor(TiXmlElement* pElem, std::string style_name, int r, int g, int b, int a)
{
	std::ostringstream color_hexa;
	color_hexa << std::setfill('0') << std::setw(2)<< std::hex << a <<
			std::setfill('0') << std::setw(2) << b <<
			std::setfill('0') << std::setw(2) << g <<
			std::setfill('0') << std::setw(2) << r;

	pElem->LinkEndChild(new TiXmlElement(style_name));
	TiXmlElement* pStyleElem = pElem->LastChild()->ToElement();
	pStyleElem->LinkEndChild(new TiXmlElement("color"));
	pStyleElem->LastChild()->LinkEndChild(new TiXmlText(color_hexa.str()));
	return pStyleElem;
}

void InsertNewStyle(TiXmlElement* pElem, std::string style_id, int r, int g, int b)
{
	pElem->LinkEndChild(new TiXmlElement("Style"));
	TiXmlElement* pStyleElem = pElem->LastChild()->ToElement();
	pStyleElem->SetAttribute("id", style_id);

	TiXmlElement* pLineStyleElem = CreateStyleWithColor(pStyleElem, "LineStyle", 50,50,255,255);
	pLineStyleElem->LinkEndChild(new TiXmlElement("width"));
	std::ostringstream width_str;
	width_str << LINE_WIDTH;
	pLineStyleElem->LastChild()->LinkEndChild(new TiXmlText(width_str.str()));

	TiXmlElement* pPolyStyleElem = CreateStyleWithColor(pStyleElem, "PolyStyle", 100,255,100,125);
//	pPolyStyleElem->LinkEndChild(new TiXmlElement("outline"));
//	std::ostringstream outline_str;
//	outline_str << 0;
//	pPolyStyleElem->LastChild()->LinkEndChild(new TiXmlText(outline_str.str()));
}

void InitializeStyleForSateliteNumbers(TiXmlElement* pElem)
{
	double r=0.3, g=0.95, b=0.3;
	InsertNewStyle(pElem, "Area_Style", r*255.0,g*255.0,b*255.0);
}

void CreateLinePlaceMark(std::vector<GPS_Point>& line)
{
	if(line.size() == 0 ) return;

	pHeadElem->LinkEndChild(new TiXmlElement("Placemark"));
	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("name"));
	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlText("Map Area"));

	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("styleUrl"));
	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlText("Area_Style"));

	pHeadElem->LastChild()->LinkEndChild(new TiXmlElement("Polygon"));

	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("tessellate"));
	pHeadElem->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlText("1"));

	pHeadElem->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("outerBoundaryIs"));
	pHeadElem->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("LinearRing"));
	pHeadElem->LastChild()->LastChild()->LastChild()->LastChild()->LinkEndChild(new TiXmlElement("coordinates"));

	TiXmlElement* pCoordsElem = pHeadElem->LastChild()->LastChild()->LastChild()->LastChild()->LastChild()->ToElement();

	std::ostringstream coords ;
	coords.precision(16);
	for(unsigned int i=0 ; i < line.size(); i++)
	{
		coords << line.at(i).lon << "," << line.at(i).lat << "," << line.at(i).alt << " ";
	}

	pCoordsElem->LinkEndChild(new TiXmlText(coords.str()));
}

void GetFileNameInFolder(const std::string& path, std::vector<std::string>& out_list)
{
	out_list.clear();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL)
	{
	  while ((ent = readdir (dir)) != NULL)
	  {
		  std::string str(ent->d_name);
		  if(str.compare(".") !=0 && str.compare("..") !=0)
			  out_list.push_back(path+str);
	  }
	  closedir (dir);
	}
}

void FixPathDensity(std::vector<GPS_Point>& path, const double& distanceDensity)
{
	if(path.size() == 0 || distanceDensity==0) return;

	double d = 0, a = 0;
	double margin = distanceDensity*0.01;
	double remaining = 0;
	int nPoints = 0;
	std::vector<GPS_Point> fixedPath;
	fixedPath.push_back(path.at(0));
	for(unsigned int si = 0, ei=1; ei < path.size(); )
	{
		d += hypot(path.at(ei).lat- path.at(ei-1).lat, path.at(ei).lon- path.at(ei-1).lon) + remaining;
		a = atan2(path.at(ei).lon - path.at(si).lon, path.at(ei).lat - path.at(si).lat);

		if(d < distanceDensity - margin ) // skip
		{
			ei++;
			remaining = 0;
		}
		else if(d > (distanceDensity +  margin)) // skip
		{
			GPS_Point pm = path.at(si);
			nPoints = d  / distanceDensity;
			for(int k = 0; k < nPoints; k++)
			{
				pm.lat = pm.lat + distanceDensity * cos(a);
				pm.lon = pm.lon + distanceDensity * sin(a);
				fixedPath.push_back(pm);
			}
			remaining = d - nPoints*distanceDensity;
			si++;
			path.at(si) = pm;
			d = 0;
			ei++;
		}
		else
		{
			d = 0;
			remaining = 0;
			fixedPath.push_back(path.at(ei));
			ei++;
			si = ei - 1;
		}
	}

	path = fixedPath;
}

void CompressPointCloudData(std::vector<GPS_Point>& path, double weight_data, double weight_smooth, double tolerance)
{

	if (path.size() <= 2 )
		return;

	const std::vector<GPS_Point>& path_in = path;
	std::vector<GPS_Point> smoothPath_out =  path_in;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	int size = path_in.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
			xtemp = smoothPath_out[i].lat;
			ytemp = smoothPath_out[i].lon;

			smoothPath_out[i].lat += weight_data
					* (path_in[i].lat - smoothPath_out[i].lat);
			smoothPath_out[i].lon += weight_data
					* (path_in[i].lon - smoothPath_out[i].lon);

			smoothPath_out[i].lat += weight_smooth
					* (smoothPath_out[i - 1].lat + smoothPath_out[i + 1].lat
							- (2.0 * smoothPath_out[i].lat));
			smoothPath_out[i].lon += weight_smooth
					* (smoothPath_out[i - 1].lon + smoothPath_out[i + 1].lon
							- (2.0 * smoothPath_out[i].lon));

			change += fabs(xtemp - smoothPath_out[i].lat);
			change += fabs(ytemp - smoothPath_out[i].lon);

		}
		nIterations++;
	}

	path = smoothPath_out;
}

void llaToxyz_proj(const double& lat, const double& lon, const double& alt, double& x_out, double& y_out, double& z_out)
{
	double _intern_lat = lat;
	double _intern_lon = lon;

	double _z = alt;
	double _x = DEG_TO_RAD*_intern_lat;
	double _y = DEG_TO_RAD*_intern_lon;

	printf("Befor Conversionproj : (%2.12f, %2.12f) \n" , _x, _y);

	if(pj_latlong != 0 && pj_utm !=0 )
	{
		pj_transform(pj_latlong, pj_utm, 1, 1, &_y, &_x, &_z);
		x_out = _x;
		y_out = _y;
		z_out = _z;
	}
	else
	{
		x_out = y_out = z_out = 0;
	}
}

void xyzTolla_proj(const double& x_in, const double& y_in, const double& z_in, double& lat, double& lon, double& alt)
{
	double _lat = x_in;
	double _lon = y_in;
	double _alt = z_in;

	if(pj_latlong != 0 && pj_utm !=0)
	{
		pj_transform(pj_utm,pj_latlong, 1, 1, &_lon, &_lat, &_alt);
		_lon = _lon * RAD_TO_DEG;
		_lat = _lat * RAD_TO_DEG;

		lon = _lon;
		lat = _lat;
		alt = _alt;
	}
	else
	{
		lon = lat = alt = 0;
	}
}

void LoadPointCloudData(std::vector<std::string> map_files, pcl::PointCloud<pcl::PointXYZ>& map_data)
{
	pcl::PointCloud<pcl::PointXYZ> file_data;
	std::cout << std::endl;
	for(unsigned int i=0; i < map_files.size();i++)
	{
		//std::cout << map_files.at(i) << endl;
		file_data.clear();
		pcl::io::loadPCDFile<pcl::PointXYZ> (map_files.at(i), file_data);
		map_data += file_data;
	}

	std::cout << "Point Cloud is loaded with Points: " << map_data.size() << std::endl;
}

void ConvertMap(const std::vector<std::string>& pcd_files)
{
	if(pcd_files.size()==0) return;
	if(pcd_files.at(0).size() < 5) return;

	path_id = 0;
	g_prev_sat_number = 0;
	g_line_points.clear();

	std::string fileName = pcd_files.at(0);
	std::string output_kml = fileName;
	output_kml.replace(fileName.size() - 4, fileName.size() , ".kml");

	rosbag::Bag bag;
	rosbag::View bagView;
	rosbag::View::iterator viewIterator;
	TiXmlDocument xml_doc;

	CreateTemplateDocument(xml_doc);
	pHeadElem = xml_doc.FirstChildElement()->FirstChildElement("Document");
	InitializeStyleForSateliteNumbers(pHeadElem);

	pcl::PointCloud<pcl::PointXYZ> map_data;

	std::cout << "Load PointCloud Data From Files .... " << std::endl;
	LoadPointCloudData(pcd_files, map_data);

	std::cout << "Initialize Map Polygon .... " << std::endl;
	PolygonGenerator poly(256);
	WayPoint new_center;
	std::vector<WayPoint> map_polygon = poly.EstimateClusterPolygon(map_data, new_center, 5);
	std::cout << "Contour points for this Map = " << map_polygon.size() << std::endl;

	for(unsigned int i=0; i < map_polygon.size(); i++)
	{
		g_line_points.push_back(GPS_Point(map_polygon.at(i).x,map_polygon.at(i).y, map_polygon.at(i).z));
	}


	GPS_Point p;
	std::cout << "Convert Coordinates .... " << std::endl;
	for(unsigned int i=0; i < g_line_points.size(); i++)
	{
		xyzTolla_proj(g_line_points.at(i).lon, g_line_points.at(i).lat, g_line_points.at(i).alt, p.lat, p.lon, p.alt);
		g_line_points.at(i) = p;
	}

	std::cout << "Create KML file ... " << std::endl;
	CreateLinePlaceMark(g_line_points);
	xml_doc.SaveFile(output_kml);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "map2kml");

  if(argc < 2)
  {
    std::cout << "Usage: rosrun autoware_bag_tools map2kml map_file_name.pcd \n       rosrun autoware_bag_tools map2kml map_folder/ " << std::endl;
    exit (1);
  }

  std::vector<std::string> pcd_files;

  bool bDirectory = false;

  boost::filesystem::path p_param(argv[1]);

  if(!exists(p_param))
  {
	  std::cout << "Wrong argument: Doesn't Exist" <<std::endl;
	  exit(1);
  }

  if(is_directory(p_param))
  {
	  std::vector<std::string> all_files;
	  GetFileNameInFolder(p_param.string(), all_files);

	  for (unsigned int i=0; i < all_files.size(); i++)
	  {
		  boost::filesystem::path _f(all_files.at(i));
		  std::string file_ext = _f.extension().string();
		  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::toupper);
		  if(file_ext.compare(".PCD")==0)
		  {
			  pcd_files.push_back(_f.string());
			  std::cout << "PCD File: " << _f.string() << std::endl;
		  }
	  }
  }
  else if(is_regular_file(p_param))
  {
	  std::string file_ext = p_param.extension().string();
	  std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::toupper);
	  if(file_ext.compare(".PCD")==0)
	  {
		  pcd_files.push_back(p_param.string());
		  std::cout << "PCD File: " << p_param.string() << std::endl;
	  }
  }
  else
  {
	  std::cout << "Wrong argument: Not a directory or a file !" <<std::endl;
	  exit(1);
  }

  pj_latlong = pj_init_plus("+proj=latlong");
  pj_utm = pj_init_plus("+proj=tmerc +lat_0=36.0 +lon_0=137.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs");

  ConvertMap(pcd_files);

  return 0;
}
