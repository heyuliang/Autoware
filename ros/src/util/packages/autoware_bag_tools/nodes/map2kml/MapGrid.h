/*
 * MappingGrid.h
 *
 *  Created on: September 24, 2018
 *      Author: Hatem Darweesh
 */

#ifndef MAPPINGGRID_CPY_H_
#define MAPPINGGRID_CPY_H_


#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"


namespace MAPPINGGRID_CPY_NS
{

using namespace std;

class GPS_Point
{
public:
	double lat;
	double lon;
	double alt;

	GPS_Point(double _lat, double _lon, double _alt)
	{
		lat = _lat;
		lon = _lon;
		alt = _alt;
	}

	GPS_Point()
	{
		lat = lon = alt = 0;
	}
};

class WayPoint
{
public:
	double x;
	double y;
	double z;
	double a;
	double cost;

	WayPoint(double _x, double _y, double _z, double _a)
	{
		x = _x;
		y = _y;
		z = _z;
		a = _a;
		cost = 0;
	}

	WayPoint()
	{
		x = y = z = a = cost = 0;
	}
};

class QuarterView
{
public:
	int id;
	int min_ang;
	int max_ang;
	WayPoint max_from_center;
	bool bFirst;

	QuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void InitQuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void ResetQuarterView()
	{
		bFirst = true;
	}

	bool UpdateQuarterView(const WayPoint& v)
	{
		if(v.a <= min_ang || v.a > max_ang)
			return false;

		if(bFirst)
		{
			max_from_center = v;
			bFirst = false;
		}
		else if(v.cost > max_from_center.cost)
			max_from_center = v;

		return true;
	}

	bool GetMaxPoint(WayPoint& maxPoint)
	{
		if(bFirst)
			return false;
		else
			maxPoint = max_from_center;

		return true;
	}
};

class MapCell
{
public:
	pcl::PointCloud<pcl::PointXYZRGB*> clouds;
};

class AreaBoxCell
{
public:
	int index;
	int r;
	int c;
	MapCell m_SubMap;
	bool bHasData;
	GPS_Point cell_origin;
	GPS_Point cell_avg;
	int nPoints;

	AreaBoxCell()
	{
		nPoints = 0;
		bHasData = false;
		index = 0;
		r = c = 0;
	}

	void CalcAvg()
	{
		GPS_Point sums;
		for(unsigned int i=0; i< m_SubMap.clouds.size(); i++)
		{
			sums.lat += m_SubMap.clouds.at(i)->x;
			sums.lon += m_SubMap.clouds.at(i)->y;
			sums.alt += m_SubMap.clouds.at(i)->z;
//			if(m_SubMap.clouds.at(i)->z > sums.alt) // get max z
//				sums.alt = m_SubMap.clouds.at(i)->z;
		}

		cell_avg.lat = sums.lat / (double)m_SubMap.clouds.size();
		cell_avg.lon = sums.lon / (double)m_SubMap.clouds.size();
		cell_avg.alt = sums.alt / (double)m_SubMap.clouds.size();
		//cell_avg.alt = sums.alt;

	}

};

class MappingGrid
{
public:
	MappingGrid();
	virtual ~MappingGrid();
	void LoadMap(pcl::PointCloud<pcl::PointXYZRGB>& cloudMap);
	void GetOnePathFromGridCentersLeftToRight(std::vector<GPS_Point>& points_path);
	vector<AreaBoxCell> map_cell;
	double m_width;
	double m_height;
	int m_wCells;
	int m_hCells;
	double m_res;
	GPS_Point m_origin;
	bool m_bColoredCloudData;
	pcl::PointCloud<pcl::PointXYZRGB> m_CloudMap;

	std::vector<std::vector<GPS_Point> > m_AllCenterPoints;
	std::vector<std::vector<GPS_Point> > m_AllStartPoints;

private:
	int m_point_id;

protected:
	void UpdateSubMapsWithCloudData(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
	void InsertCloudPointToSubMap(pcl::PointXYZRGB* _point);
	void CreateAreaGrid();
	AreaBoxCell* GetCellIndexFromPoint(const GPS_Point& p);
	void CompressMapGridLeftToRight();

	void FindOriginAndDimentions(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

	inline void UpdateMaxMin(const GPS_Point& p, double& _min_x,double&  _max_x,double&  _min_y,double&  _max_y)
	{
		if(p.lat > _max_x) _max_x = p.lat;

		if(p.lon > _max_y) _max_y = p.lon;

		if(p.lat < _min_x) _min_x = p.lat;

		if(p.lon < _min_y) _min_y = p.lon;
	}

	inline void UpdateMaxMin(const pcl::PointXYZRGB& p, double& _min_x,double&  _max_x,double&  _min_y,double&  _max_y)
	{
		if(p.x > _max_x) _max_x = p.x;

		if(p.y > _max_y) _max_y = p.y;

		if(p.x < _min_x) _min_x = p.x;

		if(p.y < _min_y) _min_y = p.y;
	}

	inline int get2dIndex(const int& r,const int& c, const int& w_cells)
	{
		return ((r*w_cells) + c);
	}

};

class PolygonGenerator
{

public:

	WayPoint m_Centroid;
	std::vector<QuarterView> m_Quarters;
	std::vector<WayPoint> m_Polygon;

	PolygonGenerator(int nQuarters);
	virtual ~PolygonGenerator();
	std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
	std::vector<WayPoint> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, WayPoint& new_centroid, const double& polygon_resolution = 1.0);

	 double FixNegativeAngle(const double& a)
	{
	   double angle = 0;
	   if (a < -2.0*M_PI || a > 2.0*M_PI)
		{
		   angle = fmod(a, 2.0*M_PI);
		}
	   else
		   angle = a;


	   if(angle < 0)
	   {
		   angle = 2.0*M_PI + angle;
	   }

	   return angle;
	}
};

} /* namespace SIMPLE_MAP_EDITOR_NS */

#endif /* MAPPINGGRID_CPY_H_ */
