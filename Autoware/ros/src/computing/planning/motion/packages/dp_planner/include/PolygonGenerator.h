/*
 * PolygonGenerator.h
 *
 *  Created on: Nov 2, 2016
 *      Author: ai-driver
 */

#ifndef POLYGONGENERATOR_H_
#define POLYGONGENERATOR_H_

#include "RoadNetwork.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace PlannerXNS
{

using namespace PlannerHNS;

#define QUARTERS_NUMBER 16
#define MIN_POINTS_PER_QUARTER 1
#define MIN_DISTANCE_BETWEEN_CORNERS 1.0

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
		if(v.pos.a <= min_ang || v.pos.a > max_ang)
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

class PolygonGenerator
{

public:

	GPSPoint m_Centroid;
	std::vector<QuarterView> m_Quarters;
	std::vector<GPSPoint> m_Polygon;
	PolygonGenerator();
	virtual ~PolygonGenerator();
	void CheckConvexPoligon(std::vector<WayPoint>& polygon);
	GPSPoint CalculateCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster);
	std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
	std::vector<GPSPoint> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const GPSPoint& original_centroid );
};

} /* namespace PlannerXNS */

#endif /* POLYGONGENERATOR_H_ */
