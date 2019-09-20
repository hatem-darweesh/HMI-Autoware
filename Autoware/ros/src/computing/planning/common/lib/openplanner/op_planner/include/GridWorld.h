/*
 * GridWorld.h
 *
 *  Created on: June 15, 2017
 *      Author: hatem
 */

#ifndef GridWorld_H_
#define GridWorld_H_

#include "RoadNetwork.h"
#include "cell.h"

namespace PlannerHNS
{

class GridWorld
{
  public:

    double w, inner_w; // current world width
    double h, inner_h; // current world height
    double cell_l; // cell or block length, if this is an inner cell measurements will be in meter
    double sub_cell_l;
    double origin_x , origin_y;

    int inner_start_row;
    int inner_start_col;
    int inner_end_row;
    int inner_end_col;

    bool m_bEnableInnerMap;
    bool m_bUpdatedMap;

    int wCells, nInnerWCells; // width, number of cells per row
    int hCells, nInnerHCells; // height, number of cells per column

	int m_MaxHeuristics;

	int m_DisplayResolution;

	int nSubMapRes;

	GPSPoint* delta;

    // This method map obstacles from real world space to Grid space , marking each cell or internal cells as obstacle
	void UpdateMapObstacleValue(const Obstacle& ob);
	void UpdateMapDrivablesValue(const DrivableArea& dr);
	void UpdateMapDrivablesValuePlygon(const std::vector<std::vector<GPSPoint> >& points);
	void UpdateMapObstaclesValuePlygon(const std::vector<GPSPoint>& poly, std::vector<cell*>& modifiedCell);

	cell* UpdateMapObstaclePoint(const GPSPoint& p);
	cell* UpdateThinMapObstaclePoint(const GPSPoint& p, const GPSPoint& carPos, const double& thiningTHreshold);
	cell* UpdateMapMovingObstaclePoint(const GPSPoint& p);
	cell* UpdateMapCostValueRange(const std::vector<GPSPoint>& ps, const GPSPoint& currPos, const std::vector<double>& features);
	cell* UpdateSubMapCostValue(const GPSPoint& p, const double& localize_val, const double& localize_prob);
	cell* UpdateMapCostValue(const GPSPoint& p, const double& localize_val, const double& localize_prob);
	cell* GetCellFromPointInnerMap(const GPSPoint& p);
	cell* GetCellFromPoint(const GPSPoint& p, bool bExpand = false); // return cell information from (x,y) coordinates
	cell* GetSubCellFromPoint(const GPSPoint& p); // return sub cell information from (x,y) coordinates
	cell* GetSubCellFromCell(cell* const parent, const GPSPoint& p); // return sub cell information from parent cell
	bool CheckSubCellsInTheWay(const GPSPoint& p, const GPSPoint& carPos, const double& thiningTHreshold, std::vector<cell*>& pSubCellsList);

	void ClearMap(int bMovingOnly);
	void OpenClosedCells();
	void BackupMap();

	GridWorld();
    GridWorld(double start_x, double start_y, double  map_w, double map_h, double cell_length, bool bDefaultEmpty); // initialize and construct the 2D array of the Grid cells
    void InitInnerMap(double  map_l, GridWorld* const pParentMap, const GPSPoint& center); // initialize and construct map from another map (cells will point to cells from the other map , width and hight will be maximum available limited by the parameters
    virtual ~GridWorld();

    cell* pCells;
    int nCells;

    void SaveMap(const std::string& mapFilePath, const std::string& mapName);
    void LoadMap(const std::string& mapFilePath, const GPSPoint& pos, const double& loadingRadius, const GPSPoint& mapTransformation);

    int GetSurroundingMainCells(const GPSPoint& pos, std::vector<cell*>& cells_list, double max_range=5);
    int GetSurroundingNonObstacleCells(const GPSPoint& pos, std::vector<cell*>& cells_list, double max_range=5);
    int GetSurroundingMainCellsRectangle(const GPSPoint& pos,	std::vector<cell*>& cells_list, RECTANGLE& rect);
    int GetSurroundingMainCellsCircle(const GPSPoint& pos,	std::vector<cell*>& cells_list, double radius);
    int GetSurroundingMainCellsRectangleNoObstacle(const GPSPoint& pos,	std::vector<cell*>& cells_list, RECTANGLE& rect);

    bool IsUpdated()
    {
    	return m_bUpdatedMap;
    }

    void ObservedMap()
    {
    	m_bUpdatedMap = false;
    }

  private:
    int InsidePolygon(const std::vector<GPSPoint>& polygon,const GPSPoint& p);

  };

}
#endif /* GridWorld_H_ */
