/*
 * HybridPlanner.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: hatem
 */

#include "HybridPlanner.h"
#include <iostream>

using namespace std;

namespace PlannerHNS {

HybridPlanner::HybridPlanner(GridWorld* pMap, CAR_BASIC_INFO& carInfo)
{
	m_pMap = pMap;
	m_CarInfo = carInfo;
	cost_limit = -1;
	m_iSampleRes = 3;

	double diagonalCost = sqrt(m_pMap->cell_l*m_pMap->cell_l*2.0);
	double straitCost = m_pMap->cell_l;

	double temp_big[8][3] = { { -1, 0, straitCost }, { -1, -1, diagonalCost }, { 0, -1, straitCost }, {  1,-1, diagonalCost },
			{  1, 0, straitCost }, {  1,  1, diagonalCost }, { 0,  1, straitCost }, { -1, 1, diagonalCost } }; // left, down, right, top, left down, right down, top right, left top

	for (int i = 0; i < 8; i++)
	{
		delta.push_back(WayPoint(temp_big[i][0], temp_big[i][1],0,0));
		delta.at(i).cost = temp_big[i][2];
	}

	double cost;
	double abs_a;

	double angle_ratio = (m_CarInfo.max_steer_angle * 2.0) / (m_iSampleRes - 1); //radians

	for (double steer_a = -m_CarInfo.max_steer_angle; steer_a <= m_CarInfo.max_steer_angle; steer_a += angle_ratio)
	{
		abs_a = fabs(steer_a);
		cost = m_pMap->cell_l * ((abs_a / m_CarInfo.max_steer_angle));
		actions.push_back(WayPoint(0, 0, 0, steer_a));
		actions.at(actions.size()-1).cost = cost;
		actions.at(actions.size()-1).v = m_CarInfo.max_speed_forward;
		actions.at(actions.size()-1).bDir = FORWARD_DIR;
	}
}

HybridPlanner::~HybridPlanner()
{
}

double HybridPlanner::HybridAStarPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete)
{
	cell startCell, goalCell, currNode;
	if(!InitStartGoalCells(start, goal, startCell, goalCell)) return 0;

	CalculateHeuristics(startCell, goalCell);

	paths.clear();
	if(all_cell_to_delete)
		all_cell_to_delete->clear();

	m_OpenList.clear();
	m_OpenList.push_back(startCell);

	int counter = 0;
	bool bOpenNode = GetSmallest(currNode);

	while (currNode != goalCell && bOpenNode == true && counter < m_pMap->nCells)
	{
		ForwardPass(currNode);
		bOpenNode = GetSmallest(currNode);
		counter++;
	}

	currNode = m_pMap->pCells[goalCell.index];
	int row=0, col=0;
	paths.push_back(vector<WayPoint>());

	int nCounter = 0;
	while (currNode != startCell)
	{
		row = currNode.r - delta.at(currNode.action).pos.x;
		col = currNode.c - delta.at(currNode.action).pos.y;
		WayPoint wp = m_pMap->pCells[currNode.index].center;
		paths.at(0).insert(paths.at(0).begin(), wp);
		currNode = m_pMap->pCells[get2dIndex(row,col,m_pMap->wCells)];
		nCounter++;
	}

	if(paths.at(0).size() > 0)
		paths.at(0).insert(paths.at(0).begin(), start);

	return 0;
}

double HybridPlanner::AStar(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, bool bDirectional, std::vector<WayPoint*>* all_cell_to_delete)
{
	cell startCell, goalCell, currNode;
	if(!InitStartGoalCells(start, goal, startCell, goalCell)) return 0;

	CalculateHeuristics(startCell, goalCell);

	paths.clear();
	if(all_cell_to_delete)
		all_cell_to_delete->clear();

	m_OpenList.clear();
	m_OpenList.push_back(startCell);

	int counter = 0;
	bool bOpenNode = GetSmallest(currNode);

	while (currNode != goalCell && bOpenNode == true && counter < m_pMap->nCells)
	{
		ExpandNeighbours(currNode, bDirectional);
		bOpenNode = GetSmallest(currNode);
		counter++;
	}

	currNode = m_pMap->pCells[goalCell.index];
	int row=0, col=0;
	paths.push_back(vector<WayPoint>());

	int nCounter = 0;
	while (currNode != startCell)
	{
		row = currNode.r - delta.at(currNode.action).pos.x;
		col = currNode.c - delta.at(currNode.action).pos.y;
		WayPoint wp = m_pMap->pCells[currNode.index].center;
		paths.at(0).insert(paths.at(0).begin(), wp);
		currNode = m_pMap->pCells[get2dIndex(row,col,m_pMap->wCells)];
		nCounter++;
	}

	if(paths.at(0).size() > 0)
		paths.at(0).insert(paths.at(0).begin(), start);

	return 0;
}

double HybridPlanner::AStarPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete)
{
	return AStar(start, goal, paths, false, all_cell_to_delete);
}

double HybridPlanner::AStarDirectionalPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete)
{
	return AStar(start, goal, paths, true, all_cell_to_delete);
}

bool HybridPlanner::InitStartGoalCells(const WayPoint& start, const WayPoint& goal, cell& sCell, cell& gCell)
{
	cell* pStartCell=0, *pGoalCell=0;

	pStartCell = m_pMap->GetCellFromPoint(start.pos, false);
	if(!pStartCell)
	{
		cout << "Hybrid Planner Error: Can not find start cell." << endl;
		return false;
	}

	pGoalCell = m_pMap->GetCellFromPoint(goal.pos, false);
	if(!pGoalCell)
	{
		cout << "Hybrid Planner Error: Can not find goal cell." << endl;
		return false;
	}

	m_pMap->ClearMap(-1);

	m_pMap->pCells[pStartCell->index].center = start;
	m_pMap->pCells[pStartCell->index].forwardCenter = start;
	m_pMap->pCells[pStartCell->index].backwardCenter = start;
	m_pMap->pCells[pStartCell->index].forwardCenter.bDir = FORWARD_DIR;
	m_pMap->pCells[pStartCell->index].backwardCenter.bDir = BACKWARD_DIR;
	sCell = m_pMap->pCells[pStartCell->index];

	m_pMap->pCells[pGoalCell->index].center = goal;
	m_pMap->pCells[pGoalCell->index].forwardCenter = goal;
	m_pMap->pCells[pGoalCell->index].backwardCenter = goal;
	m_pMap->pCells[pGoalCell->index].forwardCenter.bDir = FORWARD_DIR;
	m_pMap->pCells[pGoalCell->index].backwardCenter.bDir = BACKWARD_DIR;
	gCell = m_pMap->pCells[pGoalCell->index];

	return true;
}

double HybridPlanner::CalcDistance3D(const GPSPoint& p1, GPSPoint& p2)
{
	GPSPoint p_trans(p2.x - p1.x, p2.y - p1.y,0,0);
	GPSPoint p;
	p.x = (p_trans.x * cos(-p1.a) - p_trans.y * sin(-p1.a)) + p1.x;
	p.y = (p_trans.x * sin(-p1.a) + p_trans.y * cos(-p1.a)) + p1.y;

	p2.a = fabs(atan2(p1.y - p.y, p1.x - p.x));

	double a = pow(p2.a*RAD2DEG*2,2);

	double d = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

	return sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + a/d);
}

void HybridPlanner::CalculateHeuristics(const cell& start, const cell& goal)
{
	int loop_size =  m_pMap->wCells *m_pMap->hCells;
	int index = 0;
	double directional_cost = 0;
	double distance_cost = 0;
	double maxHeuristicsBackward = 0;
	double maxHeuristicsDistance = 0;
	double maxHeuristicsForward = 0;
	GPSPoint inveseCenter = start.center.pos;
	inveseCenter.a = UtilityHNS::UtilityH::InverseAngle(inveseCenter.a);

	while(index != loop_size)
	{
		if(m_pMap->pCells[index] == start || m_pMap->pCells[index] == goal || m_pMap->pCells[index].nMovingPoints>0 || m_pMap->pCells[index].nStaticPoints > 0)
		{
			index ++;
			continue;
		}

		directional_cost = CalcDistance3D(goal.center.pos, m_pMap->pCells[index].center.pos);
		if(cost_limit > 0 && directional_cost > cost_limit) directional_cost = cost_limit;
		m_pMap->pCells[index].forwardHeuristic = directional_cost;
		if(directional_cost > maxHeuristicsForward) maxHeuristicsForward = directional_cost;

		directional_cost = CalcDistance3D(inveseCenter , m_pMap->pCells[index].center.pos);
		if(cost_limit > 0 && directional_cost > cost_limit) directional_cost = cost_limit;
		m_pMap->pCells[index].backwardHeuristic = directional_cost;
		if(directional_cost > maxHeuristicsBackward) maxHeuristicsBackward = directional_cost;

		distance_cost = hypot(m_pMap->pCells[index].center.pos.y - goal.center.pos.y, m_pMap->pCells[index].center.pos.x - goal.center.pos.x);
		m_pMap->pCells[index].heuristic = distance_cost;
		if(distance_cost > maxHeuristicsDistance) maxHeuristicsDistance = distance_cost;
		index++;
	}

	if(maxHeuristicsForward == 0 || maxHeuristicsBackward == 0 || maxHeuristicsDistance ==0 )
	{
		cout << "Can't Calculate Heuristics !!" << endl;
		return;
	}

	index  = 0;
	//Normalize the heuristics
	while(index != loop_size)
	{
		m_pMap->pCells[index].forwardHeuristic = m_pMap->pCells[index].forwardHeuristic/maxHeuristicsForward;
		m_pMap->pCells[index].backwardHeuristic = m_pMap->pCells[index].backwardHeuristic/maxHeuristicsBackward;
		m_pMap->pCells[index].heuristic = m_pMap->pCells[index].heuristic/maxHeuristicsDistance;
		index++;
	}
}

void HybridPlanner::ExpandNeighbours(const cell& cell, const bool& bDirectional)
{
	int index = 0;
	int row=0, col=0;
	for (int i = 0; i < 8; i++)
	{
		row = cell.r + delta.at(i).pos.x;
		col = cell.c + delta.at(i).pos.y;
		index = get2dIndex(row, col, m_pMap->wCells);

		if(row >= 0 && col >= 0 && row < m_pMap->wCells && col < m_pMap->hCells)
		{
			if (m_pMap->pCells[index].closed == false && m_pMap->pCells[index].nStaticPoints == 0 && m_pMap->pCells[index].nMovingPoints == 0)
			{
				m_pMap->pCells[index].closed = true;
				if(bDirectional)
					m_pMap->pCells[index].heuristicValue = m_pMap->pCells[index].forwardHeuristic + m_pMap->pCells[index].backwardHeuristic + cell.heuristicValue + delta.at(i).cost;
				else
					m_pMap->pCells[index].heuristicValue =  m_pMap->pCells[index].heuristic + cell.heuristicValue  + delta.at(i).cost;

				m_pMap->pCells[index].action = i;
				m_OpenList.push_back(m_pMap->pCells[index]);
			}
		}
	}
}

bool HybridPlanner::GetSmallest(cell& cell)
{
	if (m_OpenList.size() == 0)
		return false;

	int small_index = 0;
	double smallest_val = m_OpenList.at(small_index).heuristicValue;

	for (unsigned int i = 0; i < m_OpenList.size(); i++)
	{
		if (m_OpenList.at(i).heuristicValue < smallest_val)
		{
			smallest_val = m_OpenList.at(i).heuristicValue;
			small_index = i;
		}
	}

	cell = m_OpenList.at(small_index);
	m_OpenList.erase(m_OpenList.begin() + small_index);
	m_pMap->pCells[cell.index].expanded = 1;
	return true;
}

int HybridPlanner::ForwardPass(const cell& node)
{
	double init_d = m_pMap->cell_l;
	//double inc_d = init_d;
	vector<cell> target_cells;
	for(unsigned int i = 0 ; i < actions.size(); i++)
	{
//		bool bCellFound = false;
//		while(!bCellFound)
//		{
			GPSPoint p = node.forwardCenter.pos;
			p.x += init_d *  cos(p.a);
			p.y += init_d *  sin(p.a);
			p.a += (init_d * tan(actions.at(i).pos.a))/m_CarInfo.wheel_base;

			cell* pNext = m_pMap->GetCellFromPoint(p);
			if(pNext!=0)
			{
				cell c = *pNext;
				c.forwardCenter.pos = p;
				c.forwardCenter.cost = actions.at(i).cost;
				c.heuristicValue = node.heuristicValue + c.heuristic + c.forwardHeuristic + c.backwardHeuristic + c.forwardCenter.cost;
				bool bFoundAction = false;
				for (int di = 0; di < 8; di++)
				{
					if (node.r - c.r == delta.at(di).pos.x && node.c - c.c == delta.at(di).pos.y)
					{
						c.action = di;
						bFoundAction = true;
						break;
					}
				}
				if(bFoundAction)
					target_cells.push_back(c);
			}
//		}
	}

	m_OpenList.insert(m_OpenList.end(), target_cells.begin(), target_cells.end());

	return target_cells.size();
}

} /* namespace PlannerHNS */
