/*
 * PolygonGenerator.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: ai-driver
 */

#include "PolygonGenerator.h"
#include "PlanningHelpers.h"

namespace PlannerXNS
{


PolygonGenerator::PolygonGenerator()
{
	m_Quarters = CreateQuarterViews(QUARTERS_NUMBER);
}

PolygonGenerator::~PolygonGenerator() {
	// TODO Auto-generated destructor stub
}

GPSPoint PolygonGenerator::CalculateCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster)
{
	GPSPoint c;

	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		c.x += cluster.points.at(i).x;
		c.y += cluster.points.at(i).y;
	}

	c.x = c.x/cluster.points.size();
	c.y = c.y/cluster.points.size();

	return c;
}

std::vector<GPSPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const GPSPoint& original_centroid )
{
	for(unsigned int i=0; i < m_Quarters.size(); i++)
		m_Quarters.at(i).ResetQuarterView();

	WayPoint p;
	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		p.pos.x = cluster.points.at(i).x;
		p.pos.y = cluster.points.at(i).y;
		p.pos.z = original_centroid.z;

		GPSPoint v(p.pos.x - original_centroid.x , p.pos.y - original_centroid.y, 0, 0);
		p.cost = pointNorm(v);
		p.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*RAD2DEG;

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
			m_Polygon.push_back(wp.pos);
	}

//	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_Polygon.size()>1)
	{
		bChange = false;
		GPSPoint p1 =  m_Polygon.at(m_Polygon.size()-1);
		for(unsigned int i=0; i< m_Polygon.size(); i++)
		{
			GPSPoint p2 = m_Polygon.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > MIN_DISTANCE_BETWEEN_CORNERS)
			{
				GPSPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_Polygon.insert(m_Polygon.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
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

void CheckConvexPoligon(std::vector<WayPoint>& polygon)
{

//	if(polygon.size() <= 3)
//		return;
//
//	WayPoint p1 = polygon.at(0);
//	WayPoint p3;
//	WayPoint p2;
//
//	for(int i=1; i< polygon.size()-1; i++)
//	{
//		p1 = polygon.at(i-1);
//		if(i+2 == polygon.size())
//		{
//			p2 = polygon.at(polygon.size()-1);
//			p3 = polygon.at(0);
//		}
//		else
//		{
//			p2 = polygon.at(i);
//			p3 = polygon.at(i+1);
//		}
//
//	}
}

} /* namespace PlannerXNS */
