/*
 * GridWorld.cpp
 *
 *  Created on: June 15, 2017
 *      Author: hatem
 */

#include "GridWorld.h"
#include "PlanningHelpers.h"
#include <fstream>

using namespace std;

namespace PlannerHNS
{

GridWorld::GridWorld(double start_x, double start_y, double map_w, double map_h, double cell_length, bool bDefaultEmpty)
{
	assert(cell_length > 0);
	assert(map_w>0);
	assert(map_h>0);

	m_bUpdatedMap = false;
	origin_x = start_x ;
	origin_y = start_y;
	m_DisplayResolution = 1;
	sub_cell_l = 0;
	nInnerWCells = nInnerHCells = 0;
	m_bEnableInnerMap = false;
	inner_end_col = inner_end_row = inner_h = inner_start_col = inner_start_row = inner_w = 0;
	nSubMapRes = 0;

	w = map_w;
	h = map_h;

	cell_l = cell_length;
	sub_cell_l = cell_l/(double)nSubMapRes;

	wCells =  w/cell_l;
	hCells =  h/cell_l;

	nCells = wCells*hCells;
	m_MaxHeuristics = w*h*cell_l;

	pCells =  new cell[nCells];
	GPSPoint p;
	int index = 0;

	for(int r=0; r<hCells; r++)
	{
		for(int c=0; c<wCells; c++)
		{
			index = get2dIndex(r,c,wCells);
			p.x = ((double)c * cell_l) + origin_x;
			p.y = ((double)r * cell_l) + origin_y;
			pCells[index].Initialize(p, cell_l, r, c,bDefaultEmpty);
			pCells[index].index = index;

		}
	}

	double temp[8][3] = { { -1, 0, 1 }, { 0, -1, 1 }, { 1, 0, 1 }, { 0, 1, 1 }, { -1, -1, 1.5 }, { 1, -1, 1.5 }, { 1, 1, 1.5 }, { -1, 1, 1.5 } }; // left, down, right, top, left down, right down, top right, left top
	delta = new GPSPoint[8];
	for (int i = 0; i < 8; i++)
		delta[i] = GPSPoint(temp[i][0], temp[i][1],0,0);
}

int GridWorld::InsidePolygon(const vector<GPSPoint>& polygon,const GPSPoint& p)
{
	int counter = 0;
	int i;
	double xinters;
	GPSPoint p1,p2;
	int N = polygon.size();
	if(N <=0 ) return -1;

	p1 = polygon.at(0);
	for (i=1;i<=N;i++)
	{
		p2 = polygon.at(i % N);

		if (p.y > MIN(p1.y,p2.y))
		{
			if (p.y <= MAX(p1.y,p2.y))
			{
				if (p.x <= MAX(p1.x,p2.x))
				{
					if (p1.y != p2.y)
					{
						xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
						if (p1.x == p2.x || p.x <= xinters)
							counter++;
					}
				}
			}
		}
		p1 = p2;
	}

	if (counter % 2 == 0)
		return 0;
	else
		return 1;
}

void GridWorld::OpenClosedCells()
{
	int loop_size =  nCells;
	int index = 0;
	while(index != loop_size)
	{
		pCells[index].closed = false;
		pCells[index].expanded = -1;
		index++;
	}
}

void GridWorld::ClearMap(int bMovingOnly)
{
	int loop_size =  nCells;
	int index = 0;
	while(index != loop_size)
	{
		pCells[index].Clear(bMovingOnly);
		index++;
	}
	m_bUpdatedMap = true;
}

void GridWorld::UpdateMapObstacleValue(const Obstacle& ob)
{
	GPSPoint p1, p2, p3, p4;
	p1 = ob.sp;
	p2 = ob.ep;

	if(ob.polygon.size() == 0)
	{
		int loop_size =  nCells;
		int index = 0;
		while(index != loop_size)
		{
			p3 = pCells[index].bottom_left;
			p4 = pCells[index].top_right;

			if(! ( p2.y < p3.y || p1.y > p4.y || p2.x < p3.x || p1.x > p4.x ))
			{
				pCells[index].nStaticPoints++;
				m_bUpdatedMap = true;
			}

			index++;
		}
	}
	else
	{
		vector<cell*> modList;
		UpdateMapObstaclesValuePlygon(ob.polygon, modList);
	}
}

void GridWorld::UpdateMapObstaclesValuePlygon(const vector<GPSPoint>& poly, vector<cell*>& modifiedCell)
{

	GPSPoint minP, maxP;
	cell* minC, *maxC;
	int index = 0;

	minP = poly[0];
	maxP = poly[0];

	for(unsigned int j=1; j< poly.size(); j++)
	{
		if(poly[j].x < minP.x) minP.x = poly[j].x;
		if(poly[j].y < minP.y) minP.y = poly[j].y;

		if(poly[j].x > maxP.x) maxP.x = poly[j].x;
		if(poly[j].y > maxP.y) maxP.y = poly[j].y;
	}


	minC = GetCellFromPoint(minP,false);
	maxC = GetCellFromPoint(maxP, false);

	if(!maxC || ! minC)
	{
		printf("Obstacle Polygon is outside the Map !!");
		return;
	}

	for(int r=minC->r; r<=maxC->r; r++)
	{
		for(int c=minC->c; c<=maxC->c; c++)
		{
			index = get2dIndex(r,c,wCells);
			GPSPoint bl= pCells[index].bottom_left;
			bl.x += 0.01;
			bl.y += 0.01;
			GPSPoint tr= pCells[index].top_right;
			tr.x -= 0.01;
			tr.y -= 0.01;
			if(InsidePolygon(poly, bl)==1 || InsidePolygon(poly, tr)==1)
			{

				pCells[index].nMovingPoints = 1;
				pCells[index].nStaticPoints = 1;
				//							pCells[index].forwardHeuristic = nCells;
				//							pCells[index].backwardHeuristic = nCells;
				//							pCells[index].heuristic = nCells;
				modifiedCell.push_back(&pCells[index]);
				m_bUpdatedMap = true;
				//pDrivableCells.push_back(&pCells[r][c]);
			}
		}
	}
}

void GridWorld::UpdateMapDrivablesValuePlygon(const vector<vector<GPSPoint> >& points)
{
	GPSPoint minP, maxP;
	cell* minC, *maxC;
	int index = 0;

	for(unsigned int i=0; i< points.size(); i++)
	{
		vector<GPSPoint> poly = points.at(i);
		maxP = minP = poly[0];

		for(unsigned int j=1; j< poly.size(); j++)
		{
			if(poly[j].x < minP.x) minP.x = poly[j].x;
			if(poly[j].y < minP.y) minP.y = poly[j].y;

			if(poly[j].x > maxP.x) maxP.x = poly[j].x;
			if(poly[j].y > maxP.y) maxP.y = poly[j].y;
		}


		minC = GetCellFromPoint(minP,false);
		maxC = GetCellFromPoint(maxP, false);

		for(int r=minC->r; r<maxC->r; r++)
		{
			for(int c=minC->c; c<maxC->c; c++)
			{
				index = get2dIndex(r,c,wCells);

				cell* pSub = &pCells[index];

				if(pSub->pInnerMap != 0)
				{
					pSub->InitSubCells(cell_l, sub_cell_l);
				}
				int index_sub = 0;
				//bool bUpdatedSubCell = false;
				while(index_sub < pSub->nCells)
				{
					if(InsidePolygon(poly, pSub->pInnerMap[index_sub].bottom_left)==1 || InsidePolygon(poly, pSub->pInnerMap[index_sub].top_right)==1)
					{
						pSub->pInnerMap[index_sub].nStaticPoints = 0;
						m_bUpdatedMap = true;
						//bUpdatedSubCell = true;
					}
					index_sub++;
				}


				//if(bUpdatedSubCell)
				{
					if(InsidePolygon(poly, pCells[index].bottom_left)==1 || InsidePolygon(poly, pCells[index].top_right)==1)
					{

						pCells[index].nStaticPoints = 0;
						m_bUpdatedMap = true;
						//pDrivableCells.push_back(&pCells[r][c]);
					}
				}
			}
		}
	}
}

void GridWorld::UpdateMapDrivablesValue(const DrivableArea& dr)
{
	GPSPoint p1, p2, p3, p4;
	p1 = dr.sp;
	p2 = dr.ep;

	int loop_size =  nCells;
	int index = 0;
	while(index != loop_size)
	{
		p3 = pCells[index].bottom_left;
		p4 = pCells[index].top_right;

		if(! ( p2.y < p3.y || p1.y > p4.y || p2.x < p3.x || p1.x > p4.x ))
		{
			pCells[index].nStaticPoints = 0;
			m_bUpdatedMap = true;
		}

		index++;
	}
}

void GridWorld::InitInnerMap(double  map_l, GridWorld* const pParentMap, const GPSPoint& center)
{
	int min_row, min_col, max_row, max_col;
	cell* scell = GetCellFromPoint(center);
	if(!scell) return;

	//Get First Left Cell
	double max_left = scell->c * pParentMap->cell_l;
	if(max_left < map_l)
	{
		min_row = 0;
	}
	else
	{
		GPSPoint p(center.x-map_l, center.y,0,0);
		min_row = GetCellFromPoint(p)->r;
	}

	double max_right = (pParentMap->hCells - scell->c) * pParentMap->cell_l;
	if(max_right < map_l)
	{
		max_row = pParentMap->hCells;
	}
	else
	{
		GPSPoint p(center.x+map_l, center.y,0,0);
		max_row = GetCellFromPoint(p)->r;
	}

	double max_bottom = scell->r * pParentMap->cell_l;
	if(max_bottom < map_l)
	{
		min_col = 0;
	}
	else
	{
		GPSPoint p(center.x, center.y-map_l,0,0);
		min_col = GetCellFromPoint(p)->c;
	}

	double max_top = (pParentMap->wCells- scell->r) * pParentMap->cell_l;
	if(max_top < map_l)
	{
		max_col = pParentMap->wCells;
	}
	else
	{
		GPSPoint p(center.x, center.y+map_l,0,0);
		max_col = GetCellFromPoint(p)->c;
	}

	inner_start_row = min_row;
	inner_start_col = min_col;
	inner_end_row = max_row;
	inner_end_col = max_col;
	nInnerWCells =max_col-min_col;
	nInnerHCells = max_row-min_row;
	inner_w =  nInnerWCells* pParentMap->cell_l;
	inner_h =  nInnerHCells* pParentMap->cell_l;
	cell_l = pParentMap->cell_l;
	sub_cell_l = pParentMap->sub_cell_l;


}

cell* GridWorld::UpdateMapCostValueRange(const vector<GPSPoint>& ps, const GPSPoint& currPos, const vector<double>& features)
{
	GPSPoint pos(currPos.x, currPos.y,0,0);
	cell* pC = GetCellFromPoint(pos);



	if(pC)
	{
		//Update Affected cells value from this new point
		vector<cell*> cells;
		GetSurroundingMainCells(pos, cells, 1);
		for(unsigned int i=0; i< cells.size(); i++)
		{
			cells[i]->UpdateSubCellCostValue(ps, cell_l, sub_cell_l);
			cells[i]->UpdateCostValue(ps);
			m_bUpdatedMap = true;
		}
		//only one level
		//			pC->UpdateSubCellCostValue(p, cell_l, sub_cell_l);

	}

	return pC;
}

bool GridWorld::CheckSubCellsInTheWay(const GPSPoint& p, const GPSPoint& carPos, const double& thiningThreshold, vector<cell*>& pSubCellsList)
{
	GPSPoint v(p.x - carPos.x, p.y - carPos.y,0,0);
	double v_norm = pointNorm(v);


	//Number of search iteration will be a ratio between the thining threshold   and the sub cell length
	double step_d = sub_cell_l;
	double start_d = -thiningThreshold/ 2.0;
	pSubCellsList.clear();
	cell* pSubCell = 0;
	while(start_d < thiningThreshold)
	{
		GPSPoint p_obstacle = p;
		p_obstacle.x += v.x/v_norm * start_d;
		p_obstacle.y += v.y/v_norm * start_d;
		pSubCell = GetSubCellFromPoint(p_obstacle);
		if(pSubCell && pSubCell->nStaticPoints>0)
			return true;

		pSubCellsList.push_back(pSubCell);
		start_d += step_d;
	}

	return false;
}

cell* GridWorld::UpdateThinMapObstaclePoint(const GPSPoint& p, const GPSPoint& carPos,const double& thiningTHreshold)
{
	cell* pC = GetCellFromPoint(p);
	if(pC)
	{
		vector<cell*> subcells_list;

		if(!CheckSubCellsInTheWay(p, carPos, thiningTHreshold, subcells_list))
		{
			pC->nStaticPoints++;
			cell* pSubc = GetSubCellFromCell(pC, p);
			if(pSubc)
			{
				if(pSubc->nStaticPoints<1)
				{
					pSubc->innerStaticPointsList.push_back(p);
					m_bUpdatedMap = true;
				}

				pSubc->nStaticPoints++;
			}
		}
	}
	return pC;
}

cell* GridWorld::UpdateMapObstaclePoint(const GPSPoint& p)
{
	cell* pC = GetCellFromPoint(p);
	if(pC)
	{
		if(pC->nStaticPoints < 5)
			pC->nStaticPoints++;
		cell* pSubc = GetSubCellFromCell(pC, p);
		if(pSubc)
		{
			if(pSubc->nStaticPoints<1)
			{
				pSubc->innerStaticPointsList.push_back(p);
				m_bUpdatedMap = true;
			}

			if(pSubc->nStaticPoints < 5)
				pSubc->nStaticPoints++;
		}
	}
	return pC;
}

cell* GridWorld::UpdateMapMovingObstaclePoint(const GPSPoint& p)
{
	cell* pC = GetCellFromPoint(p);
	if(pC)
	{
		if(pC->nMovingPoints < 5)
			pC->nMovingPoints++;
		cell* pSubc = GetSubCellFromCell(pC, p);
		if(pSubc)
		{
			if(pSubc->nMovingPoints<1)
			{
				pSubc->innerMovingPointsList.push_back(p);
				m_bUpdatedMap = true;
			}

			if(pSubc->nMovingPoints<5)
				pSubc->nMovingPoints++;
		}
	}
	return pC;
}

cell* GridWorld::UpdateMapCostValue(const GPSPoint& p, const double& localize_val, const double& localize_prob)
{
	cell* pC = GetCellFromPoint(p);
	if(pC)
	{
		pC->localize_val = localize_val;
		pC->localize_prob = localize_prob;
		m_bUpdatedMap = true;
	}

	return pC;
}

cell* GridWorld::UpdateSubMapCostValue(const GPSPoint& p, const double& localize_val, const double& localize_prob)
{
	cell* pC = GetCellFromPoint(p);
	if(pC)
	{

		cell* pSubc = GetSubCellFromCell(pC, p);
		if(pSubc)
		{
			pSubc->localize_val = localize_val;
			pSubc->localize_prob = localize_prob;
			m_bUpdatedMap = true;
		}
	}

	return pC;
}

cell* GridWorld::GetSubCellFromCell(cell* const parent, const GPSPoint& p)
{
	if(!parent) return 0;

	if(!parent->pInnerMap)
		parent->InitSubCells(cell_l, sub_cell_l);

	int row = floor((p.y - parent->bottom_left.y)/sub_cell_l);
	int col = floor((p.x - parent->bottom_left.x)/sub_cell_l);

	if(row>=0 && row<nSubMapRes && col >=0 && col < nSubMapRes)
		return &parent->pInnerMap[get2dIndex(row,col,nSubMapRes)];
	else
		return 0;
}

cell* GridWorld::GetSubCellFromPoint(const GPSPoint& p)
{
	cell* pMainCell = GetCellFromPoint(p);
	if(pMainCell)
	{
		if(!pMainCell->pInnerMap)
			pMainCell->InitSubCells(cell_l, sub_cell_l);

		int row = floor((p.y - pMainCell->bottom_left.y)/sub_cell_l);
		int col = floor((p.x - pMainCell->bottom_left.x)/sub_cell_l);

		return &pMainCell->pInnerMap[get2dIndex(row,col,nSubMapRes)];
	}
	else
		return 0;
}

cell* GridWorld::GetCellFromPoint(const GPSPoint& p, bool bExpand)
{

	int row = floor((p.y-origin_y) /cell_l);
	int col = floor((p.x-origin_x) /cell_l);

	if(row>=0 && row < hCells && col >=0 && col < wCells)
	{
		//		  GPSPoint _p(p.x , p.y );
		//		bool exist = pCells[get2dIndex(row,col,nColCells)].PointInRect(p);
		//
		//		if(!exist)
		//			return 0;

		//retCell = pCells[row][col];
		//retCell.center.a = p.a;
		int index = get2dIndex(row,col,wCells);
		if(index >= 0 && index < nCells)
			return &pCells[index];
		else
			printf("Error Getting Cell with Info: P(%f,%f) , C(%d,%d), index = %d", p.x, p.y, row, col, index);
	}
	else if(bExpand)
	{
		//first get extend direction and factor
		double lf=0, rf=0,tf=0,bf=0;
		int nRC=0,nCC=0;
		if(fabsf(p.x) >= 0)
			nRC= (fabsf(p.x) - 0)/cell_l + 1;

		if(fabsf(p.y) >= 0)
			nCC= (fabsf(p.y) - 0)/cell_l + 1;

		if(p.x > 0)
			rf = nRC*4.0;
		else
			lf = nRC*4.0;

		if(p.y > 0)
			tf = nCC*4.0;
		else
			bf = nCC*4.0;

	}

	return 0;
}

cell* GridWorld::GetCellFromPointInnerMap(const GPSPoint& p)
{
	int row = floor((p.y - origin_y) /cell_l);
	int col = floor((p.x - origin_x)/cell_l);

	if(row>=inner_start_row && row < inner_end_row && col >=inner_start_col && col < inner_end_col)
	{

		GPSPoint _p(p.x , p.y ,0,0);

		bool exist = pCells[get2dIndex(row,col,wCells)].PointInRect(p);

		if(!exist)
			return 0;

		//retCell = pCells[row][col];
		//retCell.center.a = p.a;
		return &pCells[get2dIndex(row,col,wCells)];
	}

	return 0;

}

void GridWorld::BackupMap()
{
}

GridWorld::GridWorld()
{
	//update_map_mutex = PTHREAD_MUTEX_INITIALIZER;
	nSubMapRes = 0;
	sub_cell_l = 0;
	nCells = 0;
	nInnerWCells = nInnerHCells = 0;
	m_bEnableInnerMap = false;
	inner_end_col = inner_end_row = inner_h = inner_start_col = inner_start_row = inner_w = 0;
	w = h = cell_l  = wCells = hCells = m_MaxHeuristics = 0;
	pCells = 0;
	m_DisplayResolution = 1;
	delta = 0;
	origin_y = 0;
	origin_x =0;
	m_bUpdatedMap  = false;
}

GridWorld::~GridWorld()
{
	if(pCells)
	{
		delete [] pCells;
		pCells = 0;
	}
	if(delta)
	{
		delete [] delta;
		delta = 0;
	}
}

int GridWorld::GetSurroundingNonObstacleCells(const GPSPoint& pos, vector<cell*>& cells_list, double max_range)
{
	int nMaxLevels = max_range/cell_l;

	int r, c;
	vector<cell*> nextLevel;
	vector<cell*> currentLevel;
	vector<cell*> tempLevel;

	cell* originalGoal = GetCellFromPoint(pos);
	if(!originalGoal) return 0;

	cell* tempCell;


	currentLevel.push_back(originalGoal);
	cells_list.push_back(originalGoal);
	int counter = 0;
	int index = 0;

	while (currentLevel.size() > 0 && nMaxLevels>0)
	{
		tempCell = currentLevel.back();
		currentLevel.pop_back();

		for (int i = 0; i < 8; i++)
		{
			counter++;
			r = tempCell->r + delta[i].x;
			c = tempCell->c + delta[i].y;
			index = get2dIndex(r,c,wCells);

			if (r >= 0 && c >= 0 && r < hCells && c < wCells)
			{
				if(pCells[index].nMovingPoints>0 || pCells[index].nStaticPoints > 0 || pCells[index].heuristic == m_MaxHeuristics)
					continue;
				//insert unique
				bool bFound = false;
				for(unsigned int j=0; j< cells_list.size();j++)
				{
					if(cells_list[j] == &pCells[index])
					{
						bFound = true;
						break;
					}
				}
				if(!bFound)
				{
					cells_list.push_back(&pCells[index]);
					nextLevel.push_back(&pCells[index]);
				}
			}
		}

		if (currentLevel.size() == 0 && nextLevel.size() > 0)
		{
			tempLevel = currentLevel;
			currentLevel = nextLevel;
			nextLevel = tempLevel;
			nMaxLevels--;
		}
	}

	return counter;
}

int GridWorld::GetSurroundingMainCellsRectangleNoObstacle(const GPSPoint& pos,	vector<cell*>& cells_list, RECTANGLE& rect)
{

	//calculate the number of levels that satisfy the max range criteria
	int nMaxLevels = hypot(rect.width, rect.length) / cell_l;

	cell* originalGoal = GetCellFromPoint(pos);

	if (!originalGoal)
		return 0;

	cells_list.push_back(originalGoal);

	if (nMaxLevels <= 1)
		return 1;

	nMaxLevels--;

	vector<pair<cell*, GPSPoint> > straitCells;
	vector<pair<cell*, GPSPoint> > diagonalCells;
	vector<pair<cell*, GPSPoint> > straitCellsL2;
	vector<pair<cell*, GPSPoint> > diagonalCellsL2;
	int r, c;
	cell* tempCell = 0;
	int counter = 1;
	int index = 0;

	GPSPoint mask;
	//first level , // left, down, right, top, left down, right down, top right, left top
	//strait degree
	for (unsigned int i = 0; i < 4; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);

		if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
		{
			straitCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
		}
	}

	//diagonal degree
	for (unsigned int i = 4; i < 8; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);
		if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
		{
			diagonalCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
		}
	}

	nMaxLevels--;
	counter++;

	while (nMaxLevels > 0)
	{
		straitCellsL2.clear();
		diagonalCellsL2.clear();
		while (straitCells.size() > 0)
		{
			mask = straitCells.back().second;
			tempCell = straitCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
			{
				straitCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 1
			if (straitCells.back().second.x == 0)
				mask.x += 1;
			else
				mask.y += 1;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 2
			if (straitCells.back().second.x == 0)
				mask.x += -2;
			else
				mask.y += -2;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			straitCells.pop_back();
		}

		//Diagonal
		while (diagonalCells.size() > 0)
		{
			mask = diagonalCells.back().second;
			tempCell = diagonalCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells) && pCells[index].nMovingPoints == 0 && pCells[index].nStaticPoints == 0 && pCells[index].heuristic != m_MaxHeuristics)
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			diagonalCells.pop_back();
		}

		nMaxLevels--;
		counter++;
		if (nMaxLevels <= 0)
			break;

		straitCells = straitCellsL2;
		diagonalCells = diagonalCellsL2;

	}

	return counter;

}

int GridWorld::GetSurroundingMainCellsRectangle(const GPSPoint& pos,	vector<cell*>& cells_list, RECTANGLE& rect)
{

	//calculate the number of levels that satisfy the max range criteria
	int nMaxLevels = hypot(rect.width, rect.length) / cell_l;

	cell* originalGoal = GetCellFromPoint(pos);

	if (!originalGoal)
		return 0;

	cells_list.push_back(originalGoal);

	if (nMaxLevels <= 1)
		return 1;

	nMaxLevels--;

	vector<pair<cell*, GPSPoint> > straitCells;
	vector<pair<cell*, GPSPoint> > diagonalCells;
	vector<pair<cell*, GPSPoint> > straitCellsL2;
	vector<pair<cell*, GPSPoint> > diagonalCellsL2;
	int r, c;
	cell* tempCell = 0;
	int counter = 1;
	int index = 0;

	GPSPoint mask;
	//first level , // left, down, right, top, left down, right down, top right, left top
	//strait degree
	for (unsigned int i = 0; i < 4; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);
		if(checkGridIndex(index, nCells))
		{
			straitCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
		}
	}

	//diagonal degree
	for (unsigned int i = 4; i < 8; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);
		if(checkGridIndex(index, nCells))
		{
			diagonalCells.push_back(make_pair(&pCells[index], mask));
			if (!pCells[index].TestWithRectangle(rect))
				cells_list.push_back(&pCells[index]);
		}
	}

	nMaxLevels--;
	counter++;

	while (nMaxLevels > 0)
	{
		straitCellsL2.clear();
		diagonalCellsL2.clear();
		while (straitCells.size() > 0)
		{
			mask = straitCells.back().second;
			tempCell = straitCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				straitCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 1
			if (straitCells.back().second.x == 0)
				mask.x += 1;
			else
				mask.y += 1;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 2
			if (straitCells.back().second.x == 0)
				mask.x += -2;
			else
				mask.y += -2;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			straitCells.pop_back();
		}

		//Diagonal
		while (diagonalCells.size() > 0)
		{
			mask = diagonalCells.back().second;
			tempCell = diagonalCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (!pCells[index].TestWithRectangle(rect))
					cells_list.push_back(&pCells[index]);
			}

			diagonalCells.pop_back();
		}

		nMaxLevels--;
		counter++;
		if (nMaxLevels <= 0)
			break;

		straitCells = straitCellsL2;
		diagonalCells = diagonalCellsL2;

	}

	return counter;

}

int GridWorld::GetSurroundingMainCellsCircle(const GPSPoint& pos,	vector<cell*>& cells_list, double radius)
{

	//calculate the number of levels that satisfy the max range criteria
	int nMaxLevels = radius * 2.0 / cell_l;

	cell* originalGoal = GetCellFromPoint(pos);

	if (!originalGoal)
		return 0;

	cells_list.push_back(originalGoal);

	if (nMaxLevels <= 1)
		return 1;

	nMaxLevels--;

	vector<pair<cell*, GPSPoint> > straitCells;
	vector<pair<cell*, GPSPoint> > diagonalCells;
	vector<pair<cell*, GPSPoint> > straitCellsL2;
	vector<pair<cell*, GPSPoint> > diagonalCellsL2;
	int r, c;
	cell* tempCell = 0;
	int counter = 1;
	int index = 0;

	GPSPoint mask;
	//first level , // left, down, right, top, left down, right down, top right, left top
	//strait degree
	for (unsigned int i = 0; i < 4; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);
		if(checkGridIndex(index, nCells))
		{
			straitCells.push_back(make_pair(&pCells[index], mask));
			if (pCells[index].TestWithCircle(pos, radius))
				cells_list.push_back(&pCells[index]);
		}
	}

	//diagonal degree
	for (unsigned int i = 4; i < 8; i++)
	{
		mask.x = delta[i].x;
		mask.y = delta[i].y;
		r = originalGoal->r + mask.x;
		c = originalGoal->c + mask.y;
		index = get2dIndex(r, c, wCells);
		if(checkGridIndex(index, nCells))
		{
			diagonalCells.push_back(make_pair(&pCells[index], mask));
			if (pCells[index].TestWithCircle(pos, radius))
				cells_list.push_back(&pCells[index]);
		}
	}

	nMaxLevels--;
	counter++;

	while (nMaxLevels > 0)
	{
		straitCellsL2.clear();
		diagonalCellsL2.clear();
		while (straitCells.size() > 0)
		{
			mask = straitCells.back().second;
			tempCell = straitCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				straitCellsL2.push_back(make_pair(&pCells[index], mask));
				if (pCells[index].TestWithCircle(pos, radius))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 1
			if (straitCells.back().second.x == 0)
				mask.x += 1;
			else
				mask.y += 1;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (pCells[index].TestWithCircle(pos, radius))
					cells_list.push_back(&pCells[index]);
			}

			//diagonal 2
			if (straitCells.back().second.x == 0)
				mask.x += -2;
			else
				mask.y += -2;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (pCells[index].TestWithCircle(pos, radius))
					cells_list.push_back(&pCells[index]);
			}

			straitCells.pop_back();
		}

		//Diagonal
		while (diagonalCells.size() > 0)
		{
			mask = diagonalCells.back().second;
			tempCell = diagonalCells.back().first;
			r = tempCell->r + mask.x;
			c = tempCell->c + mask.y;
			index = get2dIndex(r, c, wCells);
			if(checkGridIndex(index, nCells))
			{
				diagonalCellsL2.push_back(make_pair(&pCells[index], mask));
				if (pCells[index].TestWithCircle(pos, radius))
					cells_list.push_back(&pCells[index]);
			}

			diagonalCells.pop_back();
		}

		nMaxLevels--;
		counter++;
		if (nMaxLevels <= 0)
			break;

		straitCells = straitCellsL2;
		diagonalCells = diagonalCellsL2;

	}

	return counter;

}

int GridWorld::GetSurroundingMainCells(const GPSPoint& pos, vector<cell*>& cells_list, double max_range)
{

	//calculate the number of levels that satisfy the max range criteria
	int nMaxLevels = max_range/cell_l;

	int r, c;
	vector<cell*> nextLevel;
	vector<cell*> currentLevel;
	vector<cell*> tempLevel;

	cell* originalGoal = GetCellFromPoint(pos);
	if(!originalGoal) return 0;

	cell* tempCell;


	currentLevel.push_back(originalGoal);
	cells_list.push_back(originalGoal);
	int counter = 0;
	int index = 0;

	while (currentLevel.size() > 0 && nMaxLevels>0)
	{
		tempCell = currentLevel.back();
		currentLevel.pop_back();

		for (int i = 0; i < 8; i++)
		{
			counter++;
			r = tempCell->r + delta[i].x;
			c = tempCell->c + delta[i].y;
			index = get2dIndex(r,c,wCells);

			if (r >= 0 && c >= 0 && r < hCells && c < wCells)
			{
				//insert unique
				bool bFound = false;
				for(unsigned int j=0; j< cells_list.size();j++)
				{
					if(cells_list[j] == &pCells[index])
					{
						bFound = true;
						break;
					}
				}
				if(!bFound)
				{
					cells_list.push_back(&pCells[index]);
					nextLevel.push_back(&pCells[index]);
				}
			}
		}

		if (currentLevel.size() == 0 && nextLevel.size() > 0)
		{
			tempLevel = currentLevel;
			currentLevel = nextLevel;
			nextLevel = tempLevel;
			nMaxLevels--;
		}
	}

	return counter;

}

void GridWorld::SaveMap(const string& mapFilePath, const string& mapName)
{
	ofstream f(mapFilePath.c_str(),ios::out);
	if(!f.is_open())
	{
		printf("\n Can't Open Map File to Save!, %s", mapFilePath.c_str());
		return;
	}
	f.precision(8);

	if(nCells>0)
	{
		int loop_size =  nCells;
		int index = 0;
		while(index != loop_size)
		{
			if(pCells[index].nStaticPoints > 0 )
			{
				int subindex = 0;
				int sub_loop_size = pCells[index].nCells;
				while(subindex != sub_loop_size)
				{
					if(pCells[index].pInnerMap[subindex].nStaticPoints > 0)
					{
						for(unsigned int p=0; p<pCells[index].pInnerMap[subindex].innerStaticPointsList.size(); p++)
						{
							f<<pCells[index].pInnerMap[subindex].innerStaticPointsList[p].x<<","<<pCells[index].pInnerMap[subindex].innerStaticPointsList[p].y<<" ";
						}
						f<<endl;
					}

					subindex++;
				}
			}

			index++;
		}
	}

	f.close();

	//save Values Map
	//	  string cost_file = mapFilePath + "_cost.grd";
	//	  ofstream f2(cost_file.c_str(),ios::out);
	//  	  f2.precision(8);
	//  	if(nCells>0)
	//	  {
	//		  int loop_size =  nCells;
	//		  int index = 0;
	//			while(index != loop_size)
	//			{
	//				if(pCells[index].nCells>0)
	//				{
	//					if(pCells[index].localize_val>0)
	//					{
	//						f2<<"C,"<<pCells[index].center.p.x<<","<<pCells[index].center.p.y<<","<<pCells[index].localize_val << " ";
	//						f2<<endl;
	//					}
	//
	//					int subIndex = 0;
	//					while(subIndex != pCells[index].nCells)
	//					{
	//						f2<<"S,"<<pCells[index].pInnerMap[subIndex].center.p.x<<","<<pCells[index].pInnerMap[subIndex].center.p.y<<","<<pCells[index].pInnerMap[subIndex].localize_val << " ";
	//						subIndex++;
	//					}
	//					f2<<endl;
	//				}
	//
	//				index++;
	//			}
	//	  }
	//
	//  	f2.close();

}

void GridWorld::LoadMap(const string& mapFilePath, const GPSPoint& pos, const double& loadingRadius, const GPSPoint& mapTransformation)
{

	//	  GPSPoint point;
	//	  ifstream f(mapFilePath.c_str(), ios::in);
	//	  if(!f.is_open())
	//	  {
	//		  printf("\n Can't Open Map File !, %s", mapFilePath.c_str());
	//		  return;
	//	  }
	//
	//	  f.precision(8);
	//	  string token, temp, innerToken;
	//	  string strLine;
	//
	//	while(!f.eof())
	//	{
	//		getline(f, strLine);
	//		istringstream str_stream(strLine);
	//		while(getline(str_stream, innerToken, ' '))
	//		{
	//
	//			string str_x, str_y;
	//
	//			istringstream ss(innerToken);
	//
	//			getline(ss, str_x, ',');
	//			getline(ss, str_y, ',');
	//
	//			point.p.x = atof(str_x.c_str());
	//			point.p.y = atof(str_y.c_str());
	//
	//			MathUtil::CoordinateTransform(point, mapTransformation);
	//
	//			UpdateMapObstaclePoint(point);
	//		}
	//	}
	//
	//	  f.close();

	//	  string cost_file = mapFilePath + "_cost.grd";
	//	  ifstream f2(cost_file.c_str(),ios::in);
	//	  f2.precision(8);
	//	  double cost_val = 0;
	//	  while(!f2.eof())
	//	  	{
	//	  		getline(f2, strLine);
	//	  		istringstream str_stream(strLine);
	//
	//	  		while(getline(str_stream, innerToken, ' '))
	//			{
	//				string str_key, str_x, str_y, str_val;
	//				istringstream ss(innerToken);
	//				getline(ss, str_key, ',');
	//				getline(ss, str_x, ',');
	//				getline(ss, str_y, ',');
	//				getline(ss, str_val, ',');
	//
	//				point.x = atof(str_x.c_str());
	//				point.y = atof(str_y.c_str());
	//				cost_val = atof(str_val.c_str());
	//
	//				if(str_key.compare("S")==0)
	//					UpdateSubMapCostValue(point, cost_val, 0);
	//				else
	//					UpdateMapCostValue(point, cost_val, 0);
	//			}
	//
	//	  	}
	//
	//	  f2.close();
}

}
