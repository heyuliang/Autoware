/*
 * MappingGrid.cpp
 *
 *  Created on: September 24, 2018
 *      Author: Hatem Darweesh
 */

#include "MapGrid.h"
#include "float.h"


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


}
