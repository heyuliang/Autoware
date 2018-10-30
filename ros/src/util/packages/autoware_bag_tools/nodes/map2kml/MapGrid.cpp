/*
 * MappingGrid.cpp
 *
 *  Created on: September 24, 2018
 *      Author: Hatem Darweesh
 */

#include "MapGrid.h"
#include "float.h"
#include <pcl/common/centroid.h>

#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)

namespace MAPPINGGRID_CPY_NS
{

MappingGrid::MappingGrid()
{
	m_bColoredCloudData = false;
	m_res = 10.0;
	m_width = 0;
	m_height = 0;
	m_point_id = 0;
	m_hCells = 0;
	m_wCells = 0;
}

MappingGrid::~MappingGrid()
{
}


void MappingGrid::LoadMap(pcl::PointCloud<pcl::PointXYZRGB>& cloudMap)
{
	m_CloudMap = cloudMap;

	if(m_CloudMap.size() > 0)
	{
		FindOriginAndDimentions(m_CloudMap);

		CreateAreaGrid();

		cout << "Map Loaded Successfully and Grid Initialized ! " << endl;
	}
	else
	{
		cout << "Can't Initialize the Grid, No map data ! " << endl;
	}
}

AreaBoxCell* MappingGrid::GetCellIndexFromPoint(const GPS_Point& p)
{
	int col = floor((p.lat - m_origin.lat) / m_res);
	int row = floor((p.lon - m_origin.lon) / m_res);

	int index = -1;
	if(row >= 0 && row < m_height && col >=0 && col < m_width)
	{
		index = get2dIndex(row,col, m_wCells);

		if(index >= 0 && index < (int)map_cell.size())
		{
			//printf("Cell Info: P(%f,%f) , G(%d,%d), index = %d \n", p.x, p.y, col, row , index);
			return &map_cell.at((unsigned int)index);
		}
	}

	return nullptr;
}

void MappingGrid::FindOriginAndDimentions(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	//GetMax X,y and Min x,y
	double min_x = DBL_MAX, min_y = DBL_MAX;
	double max_x = -DBL_MAX, max_y = -DBL_MAX;

	for(unsigned int i=0; i < cloud.size(); i++)
	{
		UpdateMaxMin(cloud.at(i), min_x, max_x, min_y, max_y);
	}

	m_origin.lat = min_x;
	m_origin.lon = min_y;

	m_width = max_x - min_x;
	m_height = max_y - min_y;
}

void MappingGrid::CreateAreaGrid()
{
	m_wCells = m_width/m_res+1;
	m_hCells = m_height/m_res+1;
	int cells_size = m_wCells * m_hCells;
	map_cell.resize(cells_size);
	for(unsigned int r = 0 ; r < m_hCells; r++)
	{
		for(unsigned int c = 0 ; c < m_wCells; c++)
		{
			int index = get2dIndex(r,c, m_wCells);
			map_cell.at(index).index = index;
			map_cell.at(index).r = r;
			map_cell.at(index).c = c;
			map_cell.at(index).cell_origin.lat = c*m_res;
			map_cell.at(index).cell_origin.lon = r*m_res;
		}
	}

	UpdateSubMapsWithCloudData(m_CloudMap);
}

void MappingGrid::UpdateSubMapsWithCloudData(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	for(unsigned int i=0; i < map_cell.size(); i++)
	{
		map_cell.at(i).m_SubMap.clouds.clear();
	}

	for(unsigned int i=0; i < cloud.size(); i++)
	{
		InsertCloudPointToSubMap(&cloud.at(i));
	}
}

void MappingGrid::InsertCloudPointToSubMap(pcl::PointXYZRGB* _point)
{
	AreaBoxCell* pCell  = nullptr;
	if(!m_bColoredCloudData && (_point->r > 0 || _point->g > 0 || _point->b > 0))
		m_bColoredCloudData = true;

	GPS_Point p(_point->x, _point->y, _point->z);
	pCell = GetCellIndexFromPoint(p);
	if(pCell != nullptr)
	{
		pCell->bHasData = true;
		pCell->nPoints++;
		pCell->m_SubMap.clouds.push_back(_point);
	}
}

void MappingGrid::CompressMapGridLeftToRight()
{

}

void MappingGrid::GetOnePathFromGridCentersLeftToRight(std::vector<GPS_Point>& points_path)
{
	points_path.clear();
	for(unsigned int r = 0 ; r < m_hCells; r++)
	{
		for(unsigned int c = 0 ; c < m_wCells; c++)
		{
			int index = get2dIndex(r,c, m_wCells);

			if(map_cell.at(index).nPoints > 500)
			{
				map_cell.at(index).CalcAvg();
				points_path.push_back(map_cell.at(index).cell_avg);
			}
		}
	}
}

PolygonGenerator::PolygonGenerator(int nQuarters)
{
	m_Quarters = CreateQuarterViews(nQuarters);
}

PolygonGenerator::~PolygonGenerator()
{
}

std::vector<WayPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, WayPoint& new_centroid, const double& polygon_resolution)
{
	for(unsigned int i=0; i < m_Quarters.size(); i++)
			m_Quarters.at(i).ResetQuarterView();


	WayPoint sum_p_cent;
	for(unsigned int i = 0 ; i< cluster.points.size(); i++)
	{
		sum_p_cent.x += cluster.points.at(i).x;
		sum_p_cent.y += cluster.points.at(i).y;
		sum_p_cent.z += cluster.points.at(i).z;
	}

	WayPoint original_centroid;
	if(cluster.points.size() > 0)
	{
		original_centroid.x = sum_p_cent.x / (double)cluster.points.size();
		original_centroid.y = sum_p_cent.y / (double)cluster.points.size();
		original_centroid.z = sum_p_cent.z / (double)cluster.points.size();
	}

	WayPoint p;
	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		p.x = cluster.points.at(i).x;
		p.y = cluster.points.at(i).y;
		p.z = original_centroid.z;
		WayPoint v(p.x - original_centroid.x , p.y - original_centroid.y, 0, 0);
		p.cost = pointNorm(v);
		p.a = FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);
		for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
		{
			if(m_Quarters.at(j).UpdateQuarterView(p))
				break;
		}
	}

	m_Polygon.clear();
	WayPoint wp;
	for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
	{
		if(m_Quarters.at(j).GetMaxPoint(wp))
			m_Polygon.push_back(wp);
	}

//	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_Polygon.size()>1)
	{
		bChange = false;
		WayPoint p1 =  m_Polygon.at(m_Polygon.size()-1);
		for(unsigned int i=0; i< m_Polygon.size(); i++)
		{
			WayPoint p2 = m_Polygon.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > polygon_resolution)
			{
				WayPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_Polygon.insert(m_Polygon.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
	WayPoint sum_p;
	for(unsigned int i = 0 ; i< m_Polygon.size(); i++)
	{
		sum_p.x += m_Polygon.at(i).x;
		sum_p.y += m_Polygon.at(i).y;
	}

	new_centroid = original_centroid;

	if(m_Polygon.size() > 0)
	{
		new_centroid.x = sum_p.x / (double)m_Polygon.size();
		new_centroid.y = sum_p.y / (double)m_Polygon.size();
	}

	return m_Polygon;

}

std::vector<QuarterView> PolygonGenerator::CreateQuarterViews(const int& nResolution)
{
	std::vector<QuarterView> quarters;
	if(nResolution <= 0)
		return quarters;

	double range = 360.0 / nResolution;
	double angle = 0;
	for(int i = 0; i < nResolution; i++)
	{
		QuarterView q(angle, angle+range, i);
		quarters.push_back(q);
		angle+=range;
	}

	return quarters;
}

}
