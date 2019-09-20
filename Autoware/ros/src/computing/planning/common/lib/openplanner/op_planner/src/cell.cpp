/*
 * cell.cpp
 *
 *  Created on: June 15, 2017
 *      Author: hatem
 */

#include "cell.h"
#include "PlanningHelpers.h"
#include <fstream>

using namespace std;

namespace PlannerHNS
{

cell::cell(int innerCellRes)
{
	wCells = innerCellRes;
	hCells = innerCellRes;
	nCells = wCells*hCells;

	index = 0;
	r=0;
	c=0;
	pInnerMap = 0;
	heuristicValue = 0;
	forwardHeuristic = 0;
	backwardHeuristic = 0;
	heuristic = 0;
	expanded = -1;
	value = 0;
	action = -1;
	forward_heuristicValue = 0;
	backward_heuristicValue = 0;
	bDir = STANDSTILL_DIR;
	closed = false;
	nMovingPoints = 0;
	nStaticPoints = 0;
	localize_val = 0;
	localize_prob = 0;
}

cell::~cell()
{
	if(pInnerMap)
		delete [] pInnerMap;
}

void cell::ClearSubCells(bool bMovingOnly)
{
	for(int i=0; i<nCells; i++)
	{
		if(!bMovingOnly)
		{
			pInnerMap[i].nStaticPoints = 0;
			pInnerMap[i].innerStaticPointsList.clear();
		}
		pInnerMap[i].nMovingPoints = 0;
		pInnerMap[i].innerMovingPointsList.clear();
	}
}

void cell::Clear(int bMovingOnly)
{
	heuristicValue = 0;
	heuristic = 0;
	forwardHeuristic  = 0;
	backwardHeuristic  = 0;
	forward_heuristicValue = 0;
	backward_heuristicValue = 0;
	expanded = -1;
	value = 0;
	closed = false;
	action = -1;
	bDir = STANDSTILL_DIR;
	if(bMovingOnly == 1)
	{
		if(nMovingPoints>0)
		{
			nMovingPoints = 0;
			ClearSubCells(true);
		}
	}
	else if(bMovingOnly == 0)
	{
		if(nMovingPoints>0 || nStaticPoints>0)
		{
			nMovingPoints = 0;
			nStaticPoints = 0;
			ClearSubCells(false);
		}
	}
}

void cell::InitSubCells(double cell_l, double sub_cell_l)
{
	pInnerMap =  new cell[nCells];
	GPSPoint p;
	int index = 0;

	for(int rr=0; rr<hCells; rr++)
	{
		for(int cc=0; cc<wCells; cc++)
		{
			index = get2dIndex(rr,cc,wCells);
			p.x = this->bottom_left.x +  ((double)cc * sub_cell_l );
			p.y = this->bottom_left.y + ((double)rr * sub_cell_l );
			pInnerMap[index].Initialize(p, sub_cell_l, rr, cc,true);
			pInnerMap[index].index = index;
		}
	}
}

void cell::UpdateCostValue(const vector<GPSPoint>& ps)
{
	index = 0;
	double cost = 0;
	for(unsigned int i=0; i<ps.size() ; i++)
	{
		cost += sqrt(distance2points(center.pos, ps[i]));
	}
	if(localize_val==0)
		localize_val = cost;
	else
		localize_val = (localize_val + cost) / 2.0;
}

void cell::UpdateSubCellCostValue(const vector<GPSPoint>& ps, const double& cell_l, const double& sub_cell_l)
{
	if(!pInnerMap)
		InitSubCells(cell_l, sub_cell_l);

	index = 0;
	double cost = 0;
	while(index < nCells)
	{
		cost = 0;
		for(unsigned int i=0; i<ps.size() ; i++)
		{
			cost += sqrt(distance2points(pInnerMap[index].center.pos, ps[i]));
		}
		if(pInnerMap[index].localize_val == 0)
			pInnerMap[index].localize_val = cost;
		else
			pInnerMap[index].localize_val = (pInnerMap[index].localize_val+cost)/2.0;

		index++;
	}

}

void cell::Initialize(GPSPoint bottom_l, double cell_l, int row, int col, bool bDefaultEmpty)
{
	double half = cell_l / 2.0;
	center.pos.x = bottom_l.x + half;
	center.pos.y = bottom_l.y + half;
	bottom_left = bottom_l;
	top_right.x = bottom_left.x + cell_l;
	top_right.y = bottom_left.y + cell_l;
	bottom_right.x = top_right.x;
	bottom_right.y = bottom_left.y;
	top_left.x = bottom_left.x;
	top_left.y = top_right.y;
	nMovingPoints = !bDefaultEmpty;
	nStaticPoints = !bDefaultEmpty;
	r = row;
	c = col;
}

bool cell::operator==(const cell& cell)
		  {
	if((this->r == cell.r && this->c == cell.c) || this->index == cell.index)
		return true;
	else
		return false;
		  }

bool cell::operator!=(const cell& cell)
		  {
	if((this->r != cell.r || this->c != cell.c) || this->index != cell.index)
		return true;
	else
		return false;
		  }

inline bool cell::HitTest(const GPSPoint& p)
{

	bool bHit = PointInRect(p);

	if(pInnerMap && bHit)
	{
		for(int i=0; i<nCells; i++)
		{
			if(pInnerMap[i].PointInRect(p) == true) return true;
		}
	}

	return bHit;
}

bool cell::TestWithRectangle(RECTANGLE& rec)
{
	if(!rec.PointInRect(bottom_left))
		return true;
	if(!rec.PointInRect(bottom_right))
		return true;
	if(!rec.PointInRect(top_right))
		return true;
	if(!rec.PointInRect(top_left))
		return true;

	return false;
}

bool cell::TestWithCircle(GPSPoint _center, double width)
{
	if(distance2points(center.pos, _center) <= width)
		return true;
	else
		return false;
}
void cell::SaveCell(ostream& f)
{
	//	 f<<"#cell:"<<r<<c<<index<nPoints<<bottom_left.x<<bottom_left.y<<top_right.x<<top_right.y;
	//	 f<<endl;
	//	 if(pInnerMap)
	//	 {
	//		 f<<"#InnerMap:";
	//		 for(int i=0; i<nCells; i++)
	//		   {
	//			 pInnerMap[i].SaveCell(f);
	//		   }
	//	 }

}

void cell::LoadCell(ifstream& f)
{

}

}
