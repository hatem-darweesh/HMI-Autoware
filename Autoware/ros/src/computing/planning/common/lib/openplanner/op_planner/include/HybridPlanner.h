/*
 * HybridPlanner.h
 *
 *  Created on: Jun 15, 2017
 *      Author: hatem
 */

#ifndef HYBRIDPLANNER_H_
#define HYBRIDPLANNER_H_

#include "GridWorld.h"
#include "PlannerCommonDef.h"


namespace PlannerHNS
{

class HybridPlanner
{
public:
	HybridPlanner(GridWorld* pMap, CAR_BASIC_INFO& carInfo);
	virtual ~HybridPlanner();

	double AStarPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete = 0);
	double AStarDirectionalPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete = 0);
	double HybridAStarPlan(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, std::vector<WayPoint*>* all_cell_to_delete = 0);

private:
	cell m_StartCell;
	cell m_GoalCell;
	std::vector<cell> m_OpenList;
	GridWorld* m_pMap;
	CAR_BASIC_INFO m_CarInfo;
	double cost_limit;
	std::vector<WayPoint> delta;
	std::vector<WayPoint> actions;
	int m_iSampleRes;


private:
	double AStar(const WayPoint& start, const WayPoint& goal, std::vector<std::vector<WayPoint> >& paths, bool bDirectional, std::vector<WayPoint*>* all_cell_to_delete = 0);
	bool InitStartGoalCells(const WayPoint& start, const WayPoint& goal, cell& sCell, cell& gCell);
	void CalculateHeuristics(const cell& start, const cell& goal);
	double CalcDistance3D(const GPSPoint& p1, GPSPoint& p2);
	void ExpandNeighbours(const cell& cell, const bool& bDirectional = false);
	bool GetSmallest(cell& cell);
	int ForwardPass(const cell& node);
};

} /* namespace PlannerHNS */

#endif /* HYBRIDPLANNER_H_ */
