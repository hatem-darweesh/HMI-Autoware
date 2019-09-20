/*
 *  Copyright (c) 2016, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "way_planner_core.h"

namespace WayPlannerNS {

void way_planner_core::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	int nFailedCounter = 0;
	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			if(nFailedCounter > 2)
			{
				ROS_ERROR("%s", ex.what());
			}
			ros::Duration(1.0).sleep();
			nFailedCounter ++;
		}
	}
}

way_planner_core::way_planner_core()
{
	m_pCurrGoal = 0;
	m_iCurrentGoalIndex = 0;
	m_iPrevSelectedGoalIndex = -1;
	m_bKmlMap = false;
	m_bFirstStart = false;
	m_NextAction = PlannerHNS::WAITING_ACTION;
	m_PrevAction = PlannerHNS::WAITING_ACTION;
	m_SlowDownFactor = 11.0;
	//bStartPos = false;
	//bGoalPos = false;
	//bUsingCurrentPose = false;
	nh.getParam("/way_planner/pathDensity" 			, m_params.pathDensity);
	nh.getParam("/way_planner/enableSmoothing" 		, m_params.bEnableSmoothing);
	nh.getParam("/way_planner/enableLaneChange" 	, m_params.bEnableLaneChange);
	nh.getParam("/way_planner/enableRvizInput" 		, m_params.bEnableRvizInput);
	nh.getParam("/way_planner/enableReplan" 		, m_params.bEnableReplanning);
	nh.getParam("/way_planner/enableHMI" 			, m_params.bEnableHMI);

	int iSource = 0;
	nh.getParam("/way_planner/mapSource" 			, iSource);
	if(iSource == 0)
		m_params.mapSource = MAP_AUTOWARE;
	else if (iSource == 1)
		m_params.mapSource = MAP_FOLDER;
	else if(iSource == 2)
		m_params.mapSource = MAP_KML_FILE;

	nh.getParam("/way_planner/mapFileName" 			, m_params.KmlMapPath);


	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
	pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
	pub_StartPointRviz = nh.advertise<visualization_msgs::Marker>("Global_StartPoint_rviz", 1, true);
	pub_GoalPointRviz = nh.advertise<visualization_msgs::MarkerArray>("Global_GoalPoints_rviz", 1, true);
	pub_NodesListRviz = nh.advertise<visualization_msgs::MarkerArray>("Goal_Nodes_Points_rviz", 1, true);
	pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 100, true);
	pub_TrafficInfoRviz = nh.advertise<visualization_msgs::MarkerArray>("Traffic_Lights_rviz", 1, true);
	pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("HMI_Destinations_rviz", 1, true);

#ifdef ENABLE_VISUALIZE_PLAN
	m_CurrMaxCost = 1;
	m_iCurrLevel = 0;
	m_nLevelSize = 1;
	m_bSwitch = 0;
	pub_GlobalPlanAnimationRviz = nh.advertise<visualization_msgs::MarkerArray>("AnimateGlobalPlan", 1, true);
#endif

	if(m_params.bEnableHMI)
	{
		m_AvgResponseTime = 0;
		m_SocketServer = new HMISocketServer;
		while(m_SocketServer->InitSocket(10003, 10004) < 0)
		{
			usleep(1000);
		}
	}
	else
		m_SocketServer = 0;

	/** @todo To achieve perfection , you need to start sometime */

	if(m_params.bEnableRvizInput)
	{
		sub_start_pose 	= nh.subscribe("/initialpose", 					1, &way_planner_core::callbackGetStartPose, 		this);
		sub_goal_pose 	= nh.subscribe("move_base_simple/goal", 		1, &way_planner_core::callbackGetGoalPose, 		this);
	}
	else
	{
		LoadSimulationData();
	}

	sub_current_pose 		= nh.subscribe("/current_pose", 			100,	&way_planner_core::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	nh.getParam("/way_planner/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom 			= nh.subscribe("/odom", 					100,	&way_planner_core::callbackGetRobotOdom, 	this);
	else if(bVelSource == 1)
		sub_current_velocity 	= nh.subscribe("/current_velocity",		100,	&way_planner_core::callbackGetVehicleStatus, 	this);
	else if(bVelSource == 2)
		sub_can_info 			= nh.subscribe("/can_info",		100,	&way_planner_core::callbackGetCanInfo, 	this);

	//sub_current_velocity 	= nh.subscribe("/current_velocity",			100,	&way_planner_core::callbackGetVehicleStatus, 	this);
	sub_nodes_list 			= nh.subscribe("/GlobalNodesList", 			1, 		&way_planner_core::callbackGetNodesList, 		this);

	if(m_params.mapSource == MAP_AUTOWARE)
	{
		sub_map_points 	= nh.subscribe("/vector_map_info/point", 		1, &way_planner_core::callbackGetVMPoints, 		this);
		sub_map_lanes 	= nh.subscribe("/vector_map_info/lane", 		1, &way_planner_core::callbackGetVMLanes, 		this);
		sub_map_nodes 	= nh.subscribe("/vector_map_info/node", 		1, &way_planner_core::callbackGetVMNodes, 		this);
		sup_stop_lines 	= nh.subscribe("/vector_map_info/stop_line",	1, &way_planner_core::callbackGetVMStopLines, 	this);
		sub_dtlanes 	= nh.subscribe("/vector_map_info/dtlane", 		1, &way_planner_core::callbackGetVMCenterLines,	this);
	}
}

way_planner_core::~way_planner_core()
{
	if(m_params.bEnableRvizInput)
		SaveSimulationData();

	if(m_SocketServer)
	{
		delete m_SocketServer;
		m_SocketServer = 0;
	}
}

void way_planner_core::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y, msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
	m_GoalsPos.push_back(wp);
	std::ostringstream goal_name;
	goal_name << "Destination " << m_GoalsPos.size();
	m_GoalsNames.push_back(goal_name.str());

	ROS_INFO("Received Goal Pose");
}

void way_planner_core::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x, msg->pose.pose.position.y+m_OriginPos.position.y, msg->pose.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
	ROS_INFO("Received Start pose");
}

void way_planner_core::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
}

void way_planner_core::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.twist.linear.x;
	//m_VehicleState.steer += atan(m_LocalPlanner.m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);

	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
	//	if(msg->vector.z == 0x00)
	//		m_VehicleState.shift = AW_SHIFT_POS_BB;
	//	else if(msg->vector.z == 0x10)
	//		m_VehicleState.shift = AW_SHIFT_POS_DD;
	//	else if(msg->vector.z == 0x20)
	//		m_VehicleState.shift = AW_SHIFT_POS_NN;
	//	else if(msg->vector.z == 0x40)
	//		m_VehicleState.shift = AW_SHIFT_POS_RR;

	//std::cout << "PlannerX: Read Odometry ("<< m_VehicleState.speed << ", " << m_VehicleState.steer<<")" << std::endl;
}

void way_planner_core::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.linear.x;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void way_planner_core::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleState.speed = msg->speed/3.6;
	m_VehicleState.steer = msg->angle * 0.45 / 660;
	//std::cout << "Can Info, Speed: "<< m_VehicleState.speed << ", Steering: " << m_VehicleState.steer  << std::endl;
}

void way_planner_core::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	ROS_INFO("Received Map Points");
	m_AwMap.points = msg;
	m_AwMap.bPoints = true;
}

void way_planner_core::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	ROS_INFO("Received Map Lane Array");
	m_AwMap.lanes = msg;
	m_AwMap.bLanes = true;
}

void way_planner_core::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	//ROS_INFO("Received Map Nodes");


}

void way_planner_core::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	//ROS_INFO("Received Map Stop Lines");
}

void way_planner_core::callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg)
{
	ROS_INFO("Received Map Center Lines");
	m_AwMap.dtlanes = msg;
	m_AwMap.bDtLanes = true;
}

void way_planner_core::callbackGetNodesList(const vector_map_msgs::NodeArray& msg)
{

}

void way_planner_core::UpdateRoadMap(const AutowareRoadNetwork& src_map, PlannerHNS::RoadNetwork& out_map)
{
	std::vector<UtilityHNS::AisanLanesFileReader::AisanLane> lanes;
	for(unsigned int i=0; i < src_map.lanes.data.size();i++)
	{
		UtilityHNS::AisanLanesFileReader::AisanLane l;
		l.BLID 		=  src_map.lanes.data.at(i).blid;
		l.BLID2 	=  src_map.lanes.data.at(i).blid2;
		l.BLID3 	=  src_map.lanes.data.at(i).blid3;
		l.BLID4 	=  src_map.lanes.data.at(i).blid4;
		l.BNID 		=  src_map.lanes.data.at(i).bnid;
		l.ClossID 	=  src_map.lanes.data.at(i).clossid;
		l.DID 		=  src_map.lanes.data.at(i).did;
		l.FLID 		=  src_map.lanes.data.at(i).flid;
		l.FLID2 	=  src_map.lanes.data.at(i).flid2;
		l.FLID3 	=  src_map.lanes.data.at(i).flid3;
		l.FLID4 	=  src_map.lanes.data.at(i).flid4;
		l.FNID 		=  src_map.lanes.data.at(i).fnid;
		l.JCT 		=  src_map.lanes.data.at(i).jct;
		l.LCnt 		=  src_map.lanes.data.at(i).lcnt;
		l.LnID 		=  src_map.lanes.data.at(i).lnid;
		l.Lno 		=  src_map.lanes.data.at(i).lno;
		l.Span 		=  src_map.lanes.data.at(i).span;
		l.RefVel	=  src_map.lanes.data.at(i).refvel;
		l.LimitVel	=  src_map.lanes.data.at(i).limitvel;

		//		l.LaneChgFG =  src_map.lanes.at(i).;
		//		l.LaneType 	=  src_map.lanes.at(i).blid;
		//		l.LimitVel 	=  src_map.lanes.at(i).;
		//		l.LinkWAID 	=  src_map.lanes.at(i).blid;
		//		l.RefVel 	=  src_map.lanes.at(i).blid;
		//		l.RoadSecID =  src_map.lanes.at(i).;

		lanes.push_back(l);
	}

	std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints> points;

	for(unsigned int i=0; i < src_map.points.data.size();i++)
	{
		UtilityHNS::AisanPointsFileReader::AisanPoints p;
		double integ_part = src_map.points.data.at(i).l;
		double deg = trunc(src_map.points.data.at(i).l);
		double min = trunc((src_map.points.data.at(i).l - deg) * 100.0) / 60.0;
		double sec = modf((src_map.points.data.at(i).l - deg) * 100.0, &integ_part)/36.0;
		double L =  deg + min + sec;

		deg = trunc(src_map.points.data.at(i).b);
		min = trunc((src_map.points.data.at(i).b - deg) * 100.0) / 60.0;
		sec = modf((src_map.points.data.at(i).b - deg) * 100.0, &integ_part)/36.0;
		double B =  deg + min + sec;

		p.B 		= B;
		p.Bx 		= src_map.points.data.at(i).bx;
		p.H 		= src_map.points.data.at(i).h;
		p.L 		= L;
		p.Ly 		= src_map.points.data.at(i).ly;
		p.MCODE1 	= src_map.points.data.at(i).mcode1;
		p.MCODE2 	= src_map.points.data.at(i).mcode2;
		p.MCODE3 	= src_map.points.data.at(i).mcode3;
		p.PID 		= src_map.points.data.at(i).pid;
		p.Ref 		= src_map.points.data.at(i).ref;

		points.push_back(p);
	}


	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> dts;
	for(unsigned int i=0; i < src_map.dtlanes.data.size();i++)
	{
		UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt;

		dt.Apara 	= src_map.dtlanes.data.at(i).apara;
		dt.DID 		= src_map.dtlanes.data.at(i).did;
		dt.Dir 		= src_map.dtlanes.data.at(i).dir;
		dt.Dist 	= src_map.dtlanes.data.at(i).dist;
		dt.LW 		= src_map.dtlanes.data.at(i).lw;
		dt.PID 		= src_map.dtlanes.data.at(i).pid;
		dt.RW 		= src_map.dtlanes.data.at(i).rw;
		dt.cant 	= src_map.dtlanes.data.at(i).cant;
		dt.r 		= src_map.dtlanes.data.at(i).r;
		dt.slope 	= src_map.dtlanes.data.at(i).slope;

		dts.push_back(dt);
	}

	std::vector<UtilityHNS::AisanAreasFileReader::AisanArea> areas;
	std::vector<UtilityHNS::AisanIntersectionFileReader::AisanIntersection> inters;
	std::vector<UtilityHNS::AisanLinesFileReader::AisanLine> line_data;
	std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine> stop_line_data;
	std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal> signal_data;
	std::vector<UtilityHNS::AisanVectorFileReader::AisanVector> vector_data;
	std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb> curb_data;
	std::vector<UtilityHNS::AisanRoadEdgeFileReader::AisanRoadEdge> roadedge_data;
	std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;

	PlannerHNS::GPSPoint origin;//(m_OriginPos.position.x, m_OriginPos.position.y, m_OriginPos.position.z, 0);
	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(lanes, points, dts,inters, areas, line_data,
			stop_line_data, signal_data, vector_data,curb_data, roadedge_data, conn_data, origin, out_map);
}

bool way_planner_core::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{


#ifdef ENABLE_VISUALIZE_PLAN
	if(m_PlanningVisualizeTree.size() > 0)
	{
		m_PlannerH.DeleteWaypoints(m_PlanningVisualizeTree);
		m_AccumPlanLevels.markers.clear();
		m_iCurrLevel = 0;
		m_nLevelSize = 1;
	}

	std::vector<int> predefinedLanesIds;
	double ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint,
			MAX_GLOBAL_PLAN_DISTANCE, predefinedLanesIds,
			m_Map, generatedTotalPaths, &m_PlanningVisualizeTree);

	m_pCurrGoal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPoint, m_Map);

#else
	std::vector<int> predefinedLanesIds;
	double ret = 0;
	if(m_params.bEnableHMI)
	{
		if(m_NextAction == PlannerHNS::STOP_ACTION)
		{
			generatedTotalPaths.clear();
			ret = m_PlannerH.PlanUsingDPRandom(startPoint, HIM_STOP_ACTION_DISTANCE, m_Map, generatedTotalPaths);
			m_params.bEnableReplanning = false;
		}
		else if(m_NextAction != PlannerHNS::WAITING_ACTION)
		{
			ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint,
					MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange,
					predefinedLanesIds,
					m_Map, generatedTotalPaths);

		}
//		for(unsigned int im = 0; im < m_ModifiedWayPointsCosts.size(); im++)
//			m_ModifiedWayPointsCosts.at(im)->actionCost.at(0).second = 0;

		m_ModifiedWayPointsCosts.clear();
	}
	else
	{
		ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint,
				MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange,
				predefinedLanesIds,
				m_Map, generatedTotalPaths);
	}

#endif

if(ret == 0)
{
	std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
											<< ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
	return false;
}


if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
{
	if(m_params.bEnableSmoothing)
	{
		for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
		{
			PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
			PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
		}
	}

	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
	{
		PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
		if(m_NextAction == PlannerHNS::SLOWDOWN_ACTION)
		{
			for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
				generatedTotalPaths.at(i).at(j).v = m_SlowDownFactor;
		}

		std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;

	}

	return true;
}
else
{
	std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
}
return false;
}

void way_planner_core::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths)
{
	autoware_msgs::LaneArray lane_array;
	visualization_msgs::MarkerArray pathsToVisualize;

	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
		RosHelpers::ConvertFromPlannerHToAutowarePathFormat(generatedTotalPaths.at(i), lane_array);

	std_msgs::ColorRGBA total_color;
	total_color.r = 0;
	total_color.g = 0.7;
	total_color.b = 1.0;
	total_color.a = 0.9;
	RosHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
	RosHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
	RosHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
	//RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(generatedTotalPaths, pathsToVisualize);
	pub_PathsRviz.publish(pathsToVisualize);
	if((m_bFirstStart && m_params.bEnableHMI) || !m_params.bEnableHMI)
		pub_Paths.publish(lane_array);

#ifdef OPENPLANNER_ENABLE_LOGS
	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
	{
		std::ostringstream str_out;
		str_out << UtilityHNS::UtilityH::GetHomeDirectory();
		str_out << UtilityHNS::DataRW::LoggingMainfolderName;
		str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
		str_out << "GlobalPath_";
		str_out << i;
		str_out << "_";
		PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
	}
#endif
}

void way_planner_core::VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected)
{
	visualization_msgs::MarkerArray goals_array;

	for(unsigned int i=0; i< destinations.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "HMI_Destinations";
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 2.25;
		marker.scale.y = 2.25;
		marker.scale.z = 3.25;
		marker.color.a = 0.8;
		marker.id = i;
		if(i == iSelected)
		{
			marker.color.r = 1;
			marker.color.g = 0;
			marker.color.b = 0;
		}
		else
		{
			marker.color.r = 0.5;
			marker.color.g = 0.8;
			marker.color.b = 0.5;
		}
		marker.pose.position.x = destinations.at(i).pos.x;
		marker.pose.position.y = destinations.at(i).pos.y;
		marker.pose.position.z = destinations.at(i).pos.z;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);
		goals_array.markers.push_back(marker);
	}
	pub_GoalsListRviz.publish(goals_array);
}

#ifdef ENABLE_VISUALIZE_PLAN
void way_planner_core::CreateNextPlanningTreeLevelMarker(std::vector<PlannerHNS::WayPoint*>& level, visualization_msgs::MarkerArray& markerArray, double max_cost)
{
	if(level.size() == 0 && m_pCurrGoal)
		return;

	std::vector<PlannerHNS::WayPoint*> newlevel;

	//lane_waypoint_marker.frame_locked = false;

	for(unsigned int i = 0; i < level.size(); i++)
	{
		visualization_msgs::Marker lane_waypoint_marker;
		lane_waypoint_marker.header.frame_id = "map";
		lane_waypoint_marker.header.stamp = ros::Time();
		lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
		lane_waypoint_marker.ns = "tree_levels";
		lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
		lane_waypoint_marker.scale.x = 1.0;
		lane_waypoint_marker.scale.y = 0.5;
		lane_waypoint_marker.scale.z = 0.5;
		lane_waypoint_marker.color.a = 0.8;
		lane_waypoint_marker.color.b = 1-0.0;

		float norm_cost = level.at(i)->cost / max_cost * 2.0;
		if(norm_cost <= 1.0)
		{
			lane_waypoint_marker.color.r = 1-norm_cost;
			lane_waypoint_marker.color.g = 1-1.0;
		}
		else if(norm_cost > 1.0)
		{
			lane_waypoint_marker.color.r = 1-1.0;
			lane_waypoint_marker.color.g = 1- (2.0 - norm_cost);
		}

		if(markerArray.markers.size() == 0)
			lane_waypoint_marker.id = 0;
		else
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;

		lane_waypoint_marker.pose.position.x = level.at(i)->pos.x;
		lane_waypoint_marker.pose.position.y = level.at(i)->pos.y;
		lane_waypoint_marker.pose.position.z = level.at(i)->pos.z;
		double a = UtilityHNS::UtilityH::SplitPositiveAngle(level.at(i)->pos.a);
		lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a);
		markerArray.markers.push_back(lane_waypoint_marker);

		if(level.at(i)->pLeft)
		{
			lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a + M_PI_2);
			newlevel.push_back(level.at(i)->pLeft);
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;
			markerArray.markers.push_back(lane_waypoint_marker);
		}
		if(level.at(i)->pRight)
		{
			newlevel.push_back(level.at(i)->pRight);
			lane_waypoint_marker.pose.orientation = tf::createQuaternionMsgFromYaw(a - M_PI_2);
			lane_waypoint_marker.id = markerArray.markers.at(markerArray.markers.size()-1).id + 1;
			markerArray.markers.push_back(lane_waypoint_marker);
		}

		for(unsigned int j = 0; j < level.at(i)->pFronts.size(); j++)
			if(level.at(i)->pFronts.at(j))
				newlevel.push_back(level.at(i)->pFronts.at(j));

		if(hypot(m_pCurrGoal->pos.y - level.at(i)->pos.y, m_pCurrGoal->pos.x - level.at(i)->pos.x) < 0.5)
		{
			newlevel.clear();
			break;
		}

		std::cout << "Levels: " <<  lane_waypoint_marker.id << ", pLeft:" << level.at(i)->pLeft << ", pRight:" << level.at(i)->pRight << ", nFront:" << level.at(i)->pFronts.size() << ", Cost: "<< norm_cost<< std::endl;
	}

	level = newlevel;

	//std::cout << "Levels: " <<  level.size() << std::endl;
}

#endif

bool way_planner_core::HMI_DoOneStep()
{
	double min_distance = 80;//m_AvgResponseTime * m_VehicleState.speed;
	std::vector<PlannerHNS::WayPoint*> branches;

	PlannerHNS::WayPoint startPoint;

	if(m_GoalsPos.size() > 0)
		startPoint = m_CurrentPose;


	bool bMissionCompleted = false;
	if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
	{
		PlannerHNS::RelativeInfo info;
		bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
		if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
		{
			double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
			if(remaining_distance <= ARRIVE_DISTANCE)
			{
				bMissionCompleted = true;
			}
		}
	}

	PlannerHNS::WayPoint* currOptions = 0;
	if(m_GeneratedTotalPaths.size()>0 && m_GeneratedTotalPaths.at(0).size()>0)
	{
		RosHelpers::FindIncommingBranches(m_GeneratedTotalPaths,m_CurrentPose, 30, 75, branches, currOptions);
	}

	HMI_MSG msg;
	msg.type = OPTIONS_MSG;

	if(bMissionCompleted)
	{
		msg.options.push_back(PlannerHNS::DESTINATION_REACHED);
	}
	else if(branches.size() > 0)
	{
		for(unsigned int i = 0; i< branches.size(); i++)
			msg.options.push_back(branches.at(i)->actionCost.at(0).first);

		int close_index = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_GeneratedTotalPaths.at(0), m_CurrentPose);

		for(unsigned int i=close_index+1; i < m_GeneratedTotalPaths.at(0).size(); i++)
		{
			bool bFound = false;
			for(unsigned int j=0; j< branches.size(); j++)
			{
				if(branches.at(j)->id == m_GeneratedTotalPaths.at(0).at(i).id)
				{
					currOptions = branches.at(j);
					bFound = true;
					break;
				}
			}
			if(bFound)
				break;
		}

		if(currOptions !=0 )
		{
			msg.current = currOptions->actionCost.at(0).first;
			msg.currID = currOptions->laneId;
		}
	}

	//std::cout << "options Num: " << branches.size() << ", In Message: "<< msg.options.size() << std::endl;

	msg.next_destination_id = m_iCurrentGoalIndex;
	msg.destinations = m_GoalsNames;
	//if(m_SocketServer)
		m_SocketServer->SendMSG(msg);

	double total_d = 0;
	if(m_GeneratedTotalPaths.size()>0)
		for(unsigned int iwp =1; iwp < m_GeneratedTotalPaths.at(0).size(); iwp++)
			total_d += hypot(m_GeneratedTotalPaths.at(0).at(iwp).pos.y - m_GeneratedTotalPaths.at(0).at(iwp-1).pos.y, m_GeneratedTotalPaths.at(0).at(iwp).pos.x - m_GeneratedTotalPaths.at(0).at(iwp-1).pos.x);

	HMI_MSG inc_msg;
	int bNew = 0;
	if(m_SocketServer)
		bNew = m_SocketServer->GetLatestMSG(inc_msg);
	if(bNew>0)
	{

		bool bChangeAction = false;
		for(unsigned int j = 0; j< inc_msg.options.size(); j++)
		{
			if(inc_msg.options.at(j) == PlannerHNS::START_ACTION && m_PrevAction != PlannerHNS::START_ACTION)
			{
				m_NextAction = PlannerHNS::START_ACTION;
				m_bFirstStart = true;
				m_params.bEnableReplanning = true;
				std::cout << "GO Go Go  Action ! " << inc_msg.options.at(j) <<  std::endl;
				bChangeAction = true;

				if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3 && m_VehicleState.speed == 0)
				{
					PlannerHNS::RelativeInfo info;
					bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
					if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
					{
						double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
						if(remaining_distance <= REPLANNING_DISTANCE)
						{
							if(m_GoalsPos.size() > 0)
								m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
							std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
						}
					}
				}

			}
			else if(inc_msg.options.at(j) == PlannerHNS::STOP_ACTION && m_NextAction != PlannerHNS::STOP_ACTION)
			{
				std::cout << "Stop Action ! " << std::endl;
				m_NextAction = PlannerHNS::STOP_ACTION;
				bChangeAction = true;
			}
			else if(inc_msg.options.at(j) == PlannerHNS::SLOWDOWN_ACTION && m_NextAction != PlannerHNS::SLOWDOWN_ACTION)
			{
				std::cout << "Slowdown Action ! " << std::endl;
				m_NextAction = PlannerHNS::SLOWDOWN_ACTION;
				bChangeAction = true;
			}
			else if(inc_msg.options.at(j) == PlannerHNS::CHANGE_DESTINATION)
			{
				if(inc_msg.next_destination_id > -1 && inc_msg.next_destination_id < m_GoalsPos.size())
					m_iCurrentGoalIndex = inc_msg.next_destination_id;

				if(m_iPrevSelectedGoalIndex != m_iCurrentGoalIndex)
				{
					m_NextAction = PlannerHNS::CHANGE_DESTINATION;
					std::cout << "Change Destination to " << inc_msg.next_destination_id  << std::endl;
					m_iPrevSelectedGoalIndex = m_iCurrentGoalIndex;
					bChangeAction = true;
				}
			}

			m_PrevAction = m_NextAction;
		}

		for(unsigned int i = 0; i< branches.size(); i++)
		{
			for(unsigned int j = 0; j< inc_msg.options.size(); j++)
			{
				if(branches.at(i)->actionCost.at(0).first == inc_msg.options.at(j))
				{
					bChangeAction = true;
					branches.at(i)->actionCost.at(0).second = -total_d*4.0;
					m_ModifiedWayPointsCosts.push_back(branches.at(i));

					std::cout << "Turn To : " << branches.at(i)->actionCost.at(0).first <<   std::endl;
				}


			}
		}

		return bChangeAction;
	}

	return false;
}

void way_planner_core::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr;
	startStr << m_CurrentPose.pos.x << "," << m_CurrentPose.pos.y << "," << m_CurrentPose.pos.z << "," << m_CurrentPose.pos.a << ","<< m_CurrentPose.cost << "," << 0 << ",";
	simulationDataPoints.push_back(startStr.str());

	for(unsigned int i=0; i < m_GoalsPos.size(); i++)
	{
		std::ostringstream goalStr;
		goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i+1 << ",";
		simulationDataPoints.push_back(goalStr.str());
	}

	std::string header = "X,Y,Z,A,C,V,name,";

	std::ostringstream fileName;
	fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
	fileName << "EgoCar.csv";
	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
			f << simulationDataPoints.at(i) << "\r\n";
	}

	f.close();
}

int way_planner_core::LoadSimulationData()
{
	std::ostringstream fileName;
	fileName << "EgoCar.csv";

	std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
	UtilityHNS::SimulationFileReader sfr(simuDataFileName);
	UtilityHNS::SimulationFileReader::SimulationData data;

	int nData = sfr.ReadAllData(data);
	if(nData == 0)
		return 0;

	m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
	m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));
	m_GoalsNames.push_back(data.goalPoint.name);

	for(unsigned int i=0; i < data.simuCars.size(); i++)
	{
		m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
		m_GoalsNames.push_back(data.simuCars.at(i).name);
	}

	return nData;
}

void way_planner_core::PlannerMainLoop()
{
	ros::Rate loop_rate(10);
	timespec animation_timer;
	UtilityHNS::UtilityH::GetTickCount(animation_timer);

	while (ros::ok())
	{
		ros::spinOnce();
		bool bMakeNewPlan = false;

		if(m_params.bEnableHMI)
			bMakeNewPlan = HMI_DoOneStep();

		if(m_params.mapSource == MAP_KML_FILE && !m_bKmlMap)
		{
			m_bKmlMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
			visualization_msgs::MarkerArray map_marker_array;
			RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
			pub_MapRviz.publish(map_marker_array);
		}
		else if (m_params.mapSource == MAP_FOLDER && !m_bKmlMap)
		{
			m_bKmlMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_params.KmlMapPath, m_Map, true);
			visualization_msgs::MarkerArray map_marker_array;
			RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

			pub_MapRviz.publish(map_marker_array);

		}
		else if(m_params.mapSource == MAP_AUTOWARE)
		{
			if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints)
			{
				m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;
				UpdateRoadMap(m_AwMap,m_Map);
				visualization_msgs::MarkerArray map_marker_array;
				RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
				pub_MapRviz.publish(map_marker_array);
			}
		}

		if(m_GoalsPos.size() > 0)
		{
			if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
			{
				if(m_params.bEnableReplanning && m_PrevAction != PlannerHNS::STOP_ACTION && !m_params.bEnableHMI)
				{
					PlannerHNS::RelativeInfo info;
					bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
					if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
					{
						double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
						if(remaining_distance <= REPLANNING_DISTANCE)
						{
							bMakeNewPlan = true;
							if(m_GoalsPos.size() > 0)
								m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
							std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
						}
					}
				}
			}
			else
				bMakeNewPlan = true;

			if(bMakeNewPlan )
			{


				PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
				bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);



				if(bNewPlan)
				{

					bMakeNewPlan = false;
					VisualizeAndSend(m_GeneratedTotalPaths);
#ifdef ENABLE_VISUALIZE_PLAN
					//calculate new max_cost
					if(m_PlanningVisualizeTree.size() > 1)
					{
						m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));
						m_CurrMaxCost = 0;
						for(unsigned int itree = 0; itree < m_PlanningVisualizeTree.size(); itree++)
						{
							if(m_PlanningVisualizeTree.at(itree)->cost > m_CurrMaxCost)
								m_CurrMaxCost = m_PlanningVisualizeTree.at(itree)->cost;
						}
					}
#endif
				}
			}

#ifdef ENABLE_VISUALIZE_PLAN
			if(UtilityHNS::UtilityH::GetTimeDiffNow(animation_timer) > 0.5)
			{
				UtilityHNS::UtilityH::GetTickCount(animation_timer);
				m_CurrentLevel.clear();

				for(unsigned int ilev = 0; ilev < m_nLevelSize && m_iCurrLevel < m_PlanningVisualizeTree.size() ; ilev ++)
				{
					m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(m_iCurrLevel));
					m_nLevelSize += m_PlanningVisualizeTree.at(m_iCurrLevel)->pFronts.size() - 1;
					m_iCurrLevel++;
				}


				if(m_CurrentLevel.size() == 0 && m_GeneratedTotalPaths.size() > 0)
				{
					m_bSwitch++;
					m_AccumPlanLevels.markers.clear();

					if(m_bSwitch == 2)
					{
						for(unsigned int il = 0; il < m_GeneratedTotalPaths.size(); il++)
							for(unsigned int ip = 0; ip < m_GeneratedTotalPaths.at(il).size(); ip ++)
								m_CurrentLevel.push_back(&m_GeneratedTotalPaths.at(il).at(ip));

						std::cout << "Switch On " << std::endl;

						m_bSwitch = 0;

					}
					else
					{
						for(unsigned int ilev = 0; ilev < m_PlanningVisualizeTree.size()+200 ; ilev ++)
							m_CurrentLevel.push_back(m_PlanningVisualizeTree.at(0));

						std::cout << "Switch Off " << std::endl;
					}

					CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_CurrMaxCost);
					pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
				}
				else
				{
					CreateNextPlanningTreeLevelMarker(m_CurrentLevel, m_AccumPlanLevels, m_CurrMaxCost);

					if(m_AccumPlanLevels.markers.size() > 0)
						pub_GlobalPlanAnimationRviz.publish(m_AccumPlanLevels);
				}
			}
#endif

			VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
		}

		loop_rate.sleep();
	}
}

}
