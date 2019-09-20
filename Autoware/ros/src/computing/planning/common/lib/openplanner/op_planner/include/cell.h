/*
 * cell.h
 *
 *  Created on: June 15, 2017
 *      Author: hatem
 */

#ifndef CELL_H_
#define CELL_H_

#include "RoadNetwork.h"

namespace PlannerHNS
{

#define get2dIndex(r,c,w) r*w + c
#define checkGridLimit(r,c,h,w) r >= 0 && c >= 0 && r < h && c < w
#define checkGridIndex(i, nCells) i >= 0 && i < nCells


class cell
{
public:
  int r,c,index;
  WayPoint center;
  int nCells;
  double heuristic;
  double forwardHeuristic;
  double backwardHeuristic;
  double heuristicValue;
  double forward_heuristicValue;
  double backward_heuristicValue;
  int expanded; // used in path planning
  bool closed;
  double value;
  int action;
  double localize_val;
  double localize_prob;
  std::vector<double> localize_features;
  WayPoint forwardCenter;
  WayPoint backwardCenter;
  DIRECTION_TYPE bDir;
  GPSPoint bottom_left;
  GPSPoint top_right;
  GPSPoint bottom_right;
  GPSPoint top_left;
  int nStaticPoints;
  int nMovingPoints;
  int hCells, wCells;

  cell* pInnerMap;

  std::vector<GPSPoint> innerStaticPointsList;
  std::vector<GPSPoint> innerMovingPointsList;

  std::vector<GPSPoint> path;


public:
  void InitSubCells(double cell_l, double sub_cell_l);
/**
 * @brief Clear the map contents including obstacle data if bMovingOnly parameter = -1
 * @param bMovingOnly , 1 : clear cell data and moving only points, 0 clear all data including moving and static points, -1 clear data only.
 */
  void Clear(int bMovingOnly);
  void ClearSubCells(bool bMovingOnly);

  cell(int innerCellRes = 10);

  virtual ~cell();

  /*
   * Cell initialization
   */
  void Initialize(GPSPoint bottom_left, double cell_l, int row, int col, bool bDefaultEmpty);

  /*
   * assignment operator
   */
  bool operator==(const cell& cell);

  bool operator!=(const cell& cell);

  inline bool PointInRect(const GPSPoint& p)
   {
     return p.x >= bottom_left.x && p.x <= top_right.x && p.y >= bottom_left.y && p.y <= top_right.y;
   }

  bool TestWithRectangle(RECTANGLE& rec);
  bool TestWithCircle(GPSPoint _center,double  width);
   inline bool HitTest(const GPSPoint& p);

   void UpdateSubCellCostValue(const std::vector<GPSPoint>& ps, const double& cell_l, const double& sub_cell_l);
   void UpdateCostValue(const std::vector<GPSPoint>& ps);

   void SaveCell(std::ostream& f);
   void LoadCell(std::ifstream& f);

};

}
#endif /* CELL_H_ */
