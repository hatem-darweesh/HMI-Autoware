/*
 *  Copyright (c) 2017, Nagoya University
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
#include "../include/OpenPlannerSimulator_core.h"

#include "geo_pos_conv.hh"
#include "UtilityH.h"
#include "math.h"
#include "MatrixOperations.h"
#include <geometry_msgs/PoseArray.h>
#include "PolygonGenerator.h"

using namespace std;

namespace OpenPlannerSimulatorNS
{

#define REPLANNING_DISTANCE 2
#define PLANNING_DISTANCE 50

OpenPlannerSimulator::OpenPlannerSimulator()
{

	m_bMap = false;
	bNewClusters = false;

	int iSource = 0;
	ros::NodeHandle nh("~");
	nh.getParam("mapSource" 			, iSource);
	if(iSource == 0)
		m_SimParams.mapSource = MAP_AUTOWARE;
	else if (iSource == 1)
		m_SimParams.mapSource = MAP_FOLDER;
	else if(iSource == 2)
		m_SimParams.mapSource = MAP_KML_FILE;

	nh.getParam("mapFileName" 		, m_SimParams.KmlMapPath);
	nh.getParam("id" 		, m_SimParams.strID);
	nh.getParam("id" 		, m_SimParams.id);
	nh.getParam("enableRvizPoseEst" 	, m_SimParams.bRandomStart);
	nh.getParam("enableRandomGoal" 		, m_SimParams.bRandomGoal);
	nh.getParam("enableLooper" 			, m_SimParams.bLooper);
	nh.getParam("startPoseX" 			, m_SimParams.startPose.pos.x);
	nh.getParam("startPoseY" 			, m_SimParams.startPose.pos.y);
	nh.getParam("startPoseA" 			, m_SimParams.startPose.pos.a);

	nh.getParam("meshPath" 				, m_SimParams.meshPath);
	nh.getParam("baseColorR" 			, m_SimParams.modelColor.r);
	nh.getParam("baseColorG" 			, m_SimParams.modelColor.g);
	nh.getParam("baseColorB" 			, m_SimParams.modelColor.b);

	nh.getParam("logFolder" 			, m_SimParams.logPath);

	ReadParamFromLaunchFile(m_CarInfo, m_ControlParams);

	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);
	ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE = 6.0;
	m_ObstacleTracking.m_MAX_TRACKS_AFTER_LOSING = 5;
	m_ObstacleTracking.m_DT = 0.12;
	m_ObstacleTracking.m_bUseCenterOnly = true;

	m_PredControl.Init(m_ControlParams, m_CarInfo, false, false);

	m_LocalPlanner.Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_LocalPlanner.m_SimulationSteeringDelayFactor = 0.2;

	//For rviz visualization
	std::ostringstream str_s1, str_s2, str_s3, str_s4, str_s5, str_s6;
	str_s1 << "curr_simu_pose_";
	str_s1 << m_SimParams.id;

	str_s2 << "sim_beh_txt_" << m_SimParams.id;

	str_s5 << "sim_box_pose_";
	str_s5 << m_SimParams.id;

	str_s3 << "sim_velocity_";
	str_s3 << m_SimParams.id;

	str_s4 << "safety_border_";
	str_s4 << m_SimParams.id;

	str_s6 << "simu_local_trajectory_";
	str_s6 << m_SimParams.id;

	pub_CurrPoseRviz			= nh.advertise<visualization_msgs::Marker>(str_s1.str() , 100);
	pub_SimuBoxPose				= nh.advertise<geometry_msgs::PoseArray>(str_s5.str(), 100);
	//pub_SimuVelocity			= nh.advertise<geometry_msgs::TwistStamped>(str_s3.str(), 100);
	pub_SafetyBorderRviz  		= nh.advertise<visualization_msgs::Marker>(str_s4.str(), 1);
	pub_LocalTrajectoriesRviz   = nh.advertise<visualization_msgs::MarkerArray>(str_s6.str(), 1);
	pub_BehaviorStateRviz		= nh.advertise<visualization_msgs::Marker>(str_s2.str(), 1);


	// define subscribers.
	if(m_SimParams.bRandomStart)
	{
		bInitPos = false;
		sub_initialpose 		= nh.subscribe("/initialpose", 		1, &OpenPlannerSimulator::callbackGetInitPose, 		this);

		if(!m_SimParams.bRandomGoal)
		{
			bGoalPos = false;
			sub_goalpose 		= nh.subscribe("/move_base_simple/goal", 1, &OpenPlannerSimulator::callbackGetGoalPose, 		this);
		}
		else
			bGoalPos = true;
	}
	else
	{
		bInitPos = true;
		bGoalPos = true;
		PlannerHNS::WayPoint start_p, goal_p;
		int nRecords = LoadSimulationData(start_p, goal_p);
		if(nRecords > 0)
		{
			m_SimParams.startPose.pos = start_p.pos;
			m_CarInfo.max_speed_forward = start_p.v;
		}

		if(nRecords > 1 && !m_SimParams.bRandomGoal)
		{
			m_SimParams.goalPose.pos = goal_p.pos;
		}

		//InitializeSimuCar(m_SimParams.startPose);
	}

	sub_cloudClusters 			= nh.subscribe("/cloud_clusters", 	1, &OpenPlannerSimulator::callbackGetCloudClusters, 		this);
	sub_TrafficLightSignals		= nh.subscribe("/roi_signal", 		10,	&OpenPlannerSimulator::callbackGetTrafficLightSignals, 	this);

	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
	std::cout << "OpenPlannerSimulator initialized successfully " << std::endl;
}

void OpenPlannerSimulator::ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo,
		PlannerHNS::ControllerParams& m_ControlParams)
{
	ros::NodeHandle nh("~");
	nh.getParam("width", 			m_CarInfo.width );
	nh.getParam("length", 		m_CarInfo.length );
	nh.getParam("wheelBaseLength", m_CarInfo.wheel_base );
	nh.getParam("turningRadius", m_CarInfo.turning_radius );
	nh.getParam("maxSteerAngle", m_CarInfo.max_steer_angle );

	nh.getParam("maxVelocity", m_CarInfo.max_speed_forward );
	nh.getParam("minVelocity", m_CarInfo.min_speed_forward );
	nh.getParam("minVelocity", m_CarInfo.max_speed_backword );
	nh.getParam("maxAcceleration", m_CarInfo.max_acceleration );
	nh.getParam("maxDeceleration", m_CarInfo.max_deceleration );

	nh.getParam("steeringDelay", m_ControlParams.SteeringDelay );
	nh.getParam("minPursuiteDistance", m_ControlParams.minPursuiteDistance );

	m_ControlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01); // for 3 m/s

	m_PlanningParams.maxSpeed = m_CarInfo.max_speed_forward;
	m_PlanningParams.minSpeed = m_CarInfo.min_speed_forward;
	m_PlanningParams.enableFollowing = true;
	m_PlanningParams.enableHeadingSmoothing = false;
	m_PlanningParams.enableLaneChange = false;
	m_PlanningParams.enableStopSignBehavior = false;
	m_PlanningParams.enableSwerving = false;
	m_PlanningParams.enableTrafficLightBehavior = true;
	m_PlanningParams.horizonDistance = 100;
	m_PlanningParams.horizontalSafetyDistancel = 0.1;
	m_PlanningParams.verticalSafetyDistance = 0.8;
	m_PlanningParams.maxDistanceToAvoid = 2;
	m_PlanningParams.microPlanDistance = 50;
	m_PlanningParams.minDistanceToAvoid = 4;
	m_PlanningParams.minFollowingDistance = 7;
	m_PlanningParams.pathDensity = 0.5;
	m_PlanningParams.planningDistance = 1000;
	m_PlanningParams.carTipMargin = 2;
	m_PlanningParams.rollInMargin = 10;
	m_PlanningParams.rollOutDensity = 0.5;
	m_PlanningParams.rollOutNumber = 0;

}

OpenPlannerSimulator::~OpenPlannerSimulator()
{
}

void OpenPlannerSimulator::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	if(!bInitPos)
	{
		ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

		geometry_msgs::Pose p;
		p.position.x  = msg->pose.pose.position.x + m_OriginPos.position.x;
		p.position.y  = msg->pose.pose.position.y + m_OriginPos.position.y;
		p.position.z  = msg->pose.pose.position.z + m_OriginPos.position.z;
		p.orientation = msg->pose.pose.orientation;
		m_SimParams.startPose =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z , tf::getYaw(p.orientation));

		if(m_SimParams.bRandomGoal)
		{
			SaveSimulationData();
			InitializeSimuCar(m_SimParams.startPose);
		}

		bInitPos = true;
	}
}

void OpenPlannerSimulator::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	if(!bGoalPos)
	{
		PlannerHNS::WayPoint wp;
		wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y,
				msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
		m_SimParams.goalPose = wp;
		ROS_INFO("Received Goal Pose");

		SaveSimulationData();
		InitializeSimuCar(m_SimParams.startPose);

		bGoalPos = true;
	}

}

void OpenPlannerSimulator::InitializeSimuCar(PlannerHNS::WayPoint start_pose)
{
	m_LocalPlanner.m_pCurrentBehaviorState = m_LocalPlanner.m_pInitState;
	m_LocalPlanner.m_TotalPath.clear();
	m_LocalPlanner.m_TotalOriginalPath.clear();
	m_LocalPlanner.m_Path.clear();
	m_LocalPlanner.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;
	m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
	m_LocalPlanner.FirstLocalizeMe(start_pose);
	m_LocalPlanner.LocalizeMe(0);

	cout << endl << "LocalPlannerInit: ID " << m_SimParams.strID << " , Pose = ( "  << start_pose.pos.ToString() << ")" << endl;
}

void OpenPlannerSimulator::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void OpenPlannerSimulator::callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg)
{
	m_OriginalClusters.clear();
	int nOriginalPoints=0, nContourPoints = 0;

	ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_LocalPlanner.state, m_LocalPlanner.m_CarInfo, *msg, m_OriginalClusters, nOriginalPoints, nContourPoints);
	//m_ObstacleTracking.DoOneStep(m_LocalPlanner.state, m_OriginalClusters);
	//m_TrackedClusters = m_ObstacleTracking.m_DetectedObjects;
	m_TrackedClusters = m_OriginalClusters;

	//m_nTrackObjects = m_TrackedClusters.size();
	//m_TrackingTime = UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp);
	bNewClusters = true;
}

PlannerHNS::WayPoint OpenPlannerSimulator::GetRealCenter(const PlannerHNS::WayPoint& currState)
{
	PlannerHNS::WayPoint pose_center = currState;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	PlannerHNS::Mat3 rotationMatInv(currState.pos.a);
	PlannerHNS::Mat3 translationMatInv(currState.pos.x, currState.pos.y);

	pose_center.pos = translationMat*pose_center.pos;
	pose_center.pos = rotationMat*pose_center.pos;

	pose_center.pos.x += m_CarInfo.wheel_base/3.0;

	pose_center.pos = rotationMatInv*pose_center.pos;
	pose_center.pos = translationMatInv*pose_center.pos;

	return pose_center;
}

void OpenPlannerSimulator::displayFollowingInfo(const std::vector<PlannerHNS::GPSPoint>& safety_rect, PlannerHNS::WayPoint& curr_pose)
{
  static visualization_msgs::Marker m1;
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.ns = "curr_simu_pose";
  m1.type = visualization_msgs::Marker::MESH_RESOURCE;
  m1.mesh_resource = m_SimParams.meshPath;
  m1.mesh_use_embedded_materials = true;
  m1.action = visualization_msgs::Marker::ADD;

  PlannerHNS::WayPoint pose_center = GetRealCenter(curr_pose);

  m1.pose.position.x = pose_center.pos.x;
  m1.pose.position.y = pose_center.pos.y;
  m1.pose.position.z = pose_center.pos.z;

  m1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(curr_pose.pos.a));
  m1.color = m_SimParams.modelColor;
  m1.scale.x = 1.0*m_CarInfo.length/4.2;
  m1.scale.y = 1.0*m_CarInfo.width/1.85;
  m1.scale.z = 1.0;

  pub_CurrPoseRviz.publish(m1);

  visualization_msgs::Marker lane_waypoint_marker;
  	lane_waypoint_marker.header.frame_id = "map";
  	lane_waypoint_marker.header.stamp = ros::Time();
  	lane_waypoint_marker.ns = "safety_simu_box";
  	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  	lane_waypoint_marker.scale.x = 0.2;
  	lane_waypoint_marker.scale.y = 0.2;
  	//lane_waypoint_marker.scale.z = 0.1;
  	lane_waypoint_marker.frame_locked = false;
  	lane_waypoint_marker.color.r = 0.0;
  	lane_waypoint_marker.color.g = 1.0;
  	lane_waypoint_marker.color.b = 0.0;
  	lane_waypoint_marker.color.a = 0.6;

  	for(unsigned int i=0; i < safety_rect.size(); i ++)
  	{
  		geometry_msgs::Point p1;
  		p1.x = safety_rect.at(i).x;
		p1.y = safety_rect.at(i).y;
		p1.z = safety_rect.at(i).z;
		lane_waypoint_marker.points.push_back(p1);
  	}
  	pub_SafetyBorderRviz.publish(lane_waypoint_marker);

}

void OpenPlannerSimulator::visualizePath(const std::vector<PlannerHNS::WayPoint>& path)
{
	visualization_msgs::MarkerArray markerArray;

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	std::ostringstream str_sn;
	str_sn << "simu_car_path_";
	str_sn << m_SimParams.id;
	lane_waypoint_marker.ns = str_sn.str();
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;


	std_msgs::ColorRGBA roll_color, curr_color;
	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = 1;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	lane_waypoint_marker.color.a = 0.5;
	lane_waypoint_marker.color = m_SimParams.modelColor;
	lane_waypoint_marker.frame_locked = false;

	int count = 0;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = path.at(i).pos.x;
		point.y = path.at(i).pos.y;
		point.z = path.at(i).pos.z;

		lane_waypoint_marker.points.push_back(point);
		count++;
	}

	markerArray.markers.push_back(lane_waypoint_marker);

	pub_LocalTrajectoriesRviz.publish(markerArray);
}

void OpenPlannerSimulator::ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const PlannerHNS::CAR_BASIC_INFO& car_info,
		const autoware_msgs::CloudClusterArray& clusters, std::vector<PlannerHNS::DetectedObject>& obstacles_list,
		int& nOriginalPoints, int& nContourPoints)
{
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	int nPoints = 0;
	int nOrPoints = 0;
	for(unsigned int i =0; i < clusters.clusters.size(); i++)
	{
		if((int)clusters.clusters.at(i).id == m_SimParams.id)
		{
			//std::cout << "Skip Same ID " << std::endl;
			continue;
		}

		PolygonGenerator polyGen;
		PlannerHNS::DetectedObject obj;
		obj.center.pos = PlannerHNS::GPSPoint(clusters.clusters.at(i).centroid_point.point.x,
				clusters.clusters.at(i).centroid_point.point.y,
				clusters.clusters.at(i).centroid_point.point.z,0);
				//tf::getYaw(clusters.clusters.at(i).bounding_box.pose.orientation));

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromROSMsg(clusters.clusters.at(i).cloud, point_cloud);
		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos);
		obj.w = clusters.clusters.at(i).dimensions.y;
		obj.l = clusters.clusters.at(i).dimensions.x;
		obj.h = clusters.clusters.at(i).dimensions.z;
		obj.id = 0;


		PlannerHNS::GPSPoint relative_point;
		relative_point = translationMat*obj.center.pos;
		relative_point = rotationMat*relative_point;

//		double distance_x = fabs(relative_point.x);
//		double distance_y = fabs(relative_point.y);

//		double size = (obj.w+obj.l)/2.0;
//		if(size <= 0.25 || size >= 5 || distance_y > 20.0 || distance_x > 20.0)
//			continue;

//		if(distance_x <= car_info.length && distance_y <= car_info.width/1.5) // don't detect yourself
//			continue;


		nOrPoints += point_cloud.points.size();
		nPoints += obj.contour.size();
		//std::cout << " Distance_X: " << distance_x << ", " << " Distance_Y: " << distance_y << ", " << " Size: " << size << std::endl;

		obstacles_list.push_back(obj);
	}

	nOriginalPoints = nOrPoints;
	nContourPoints =  nPoints;
}

void OpenPlannerSimulator::callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg)
{
//	std::cout << "Received Traffic Light Signals : " << msg.Signals.size() << std::endl;
	m_CurrTrafficLight.clear();
	bNewLightSignal = true;
	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = msg.Signals.at(i).signalId;
		if(msg.Signals.at(i).type == 1)
			tl.lightState = PlannerHNS::GREEN_LIGHT;
		else
			tl.lightState = PlannerHNS::RED_LIGHT;

		m_CurrTrafficLight.push_back(tl);
	}
}

void OpenPlannerSimulator::visualizeBehaviors()
{
	visualization_msgs::Marker behaviorMarker;
	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	std::ostringstream str_sn;
	str_sn << "sim_behavior_" << m_SimParams.id;
	behaviorMarker.ns = str_sn.str();
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0;
	behaviorMarker.scale.x = 1.0;
	behaviorMarker.scale.y = 1.0;
	behaviorMarker.color.a = 1.0;
	behaviorMarker.frame_locked = false;

	if(m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bTrafficIsRed)
	{
		behaviorMarker.color.r = 1.0;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 0.0;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.0;
	}
	else
	{
		behaviorMarker.color.r = 0.0;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 1.0;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.0;
	}

	geometry_msgs::Point point;

	point.x = m_LocalPlanner.state.pos.x;
	point.y = m_LocalPlanner.state.pos.y;
	point.z = m_LocalPlanner.state.pos.z+2.5;

	behaviorMarker.pose.position = point;

	behaviorMarker.id = 1;

	std::string str = "Unknown";
	switch(m_LocalPlanner.m_pCurrentBehaviorState->m_Behavior)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "Init";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Waiting";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Forward";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "Stop";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "End";
		break;
	case PlannerHNS::FOLLOW_STATE:
		str = "Follow";
		break;
	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
		str = "Swerving";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
		str = "Light Stop";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
		str = "Light Wait";
		break;
	case PlannerHNS::STOP_SIGN_STOP_STATE:
		str = "Sign Stop";
		break;
	case PlannerHNS::STOP_SIGN_WAIT_STATE:
		str = "Sign Wait";
		break;
	default:
		str = "Unknown";
		break;
	}

	std::ostringstream str_out;
	str_out << str << "(" << m_SimParams.id << ")" ;
	behaviorMarker.text = str_out.str();

	pub_BehaviorStateRviz.publish(behaviorMarker);
}

void OpenPlannerSimulator::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr, goalStr;
	startStr << m_SimParams.startPose.pos.x << "," << m_SimParams.startPose.pos.y << "," << m_SimParams.startPose.pos.z << "," << m_SimParams.startPose.pos.a << ","<< m_SimParams.startPose.cost << "," << m_CarInfo.max_speed_forward << ",";
	goalStr << m_SimParams.goalPose.pos.x << "," << m_SimParams.goalPose.pos.y << "," << m_SimParams.goalPose.pos.z << "," << m_SimParams.goalPose.pos.a << "," << 0 << "," << 0 << ",";

	simulationDataPoints.push_back(startStr.str());
	simulationDataPoints.push_back(goalStr.str());

	std::string header = "X,Y,Z,A,C,V,";

	ostringstream fileName;
	fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
	fileName << "SimuCar_";
	fileName << m_SimParams.id;
	fileName << ".csv";

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

int OpenPlannerSimulator::LoadSimulationData(PlannerHNS::WayPoint& start_p, PlannerHNS::WayPoint& goal_p)
{
	ostringstream fileName;
	fileName << "SimuCar_";
	fileName << m_SimParams.id;
	fileName << ".csv";

	string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
	UtilityHNS::SimulationFileReader sfr(simuDataFileName);
	UtilityHNS::SimulationFileReader::SimulationData data;

	int nData = sfr.ReadAllData(data);
	if(nData == 0)
		return 0;

	start_p = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
	goal_p = PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a);

	start_p.v = data.startPoint.v;
	start_p.cost = data.startPoint.c;
	return nData;
}

void OpenPlannerSimulator::PlannerMainLoop()
{

	ros::Rate loop_rate(25);
	//PlannerHNS::WayPoint defaultStart(3704.15014648,-99459.0743942, 88, 3.12940141294);
	PlannerHNS::BehaviorState currBehavior;
	PlannerHNS::VehicleState  currStatus;
	PlannerHNS::VehicleState  desiredStatus;

	while (ros::ok())
	{
		ros::spinOnce();

		bool bMakeNewPlan = false;

		if(m_SimParams.mapSource == MAP_KML_FILE && !m_bMap)
		{
			m_bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_SimParams.KmlMapPath, m_Map);
			if(!m_SimParams.bRandomStart)
				InitializeSimuCar(m_SimParams.startPose);
		}
		else if (m_SimParams.mapSource == MAP_FOLDER && !m_bMap)
		{
			m_bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_SimParams.KmlMapPath, m_Map, true);
			if(!m_SimParams.bRandomStart)
				InitializeSimuCar(m_SimParams.startPose);
		}

		if(m_bMap && bInitPos && (!m_SimParams.bRandomGoal && bGoalPos))
		{
			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			//Global Planning Step
			if(m_LocalPlanner.m_TotalOriginalPath.size() > 0 && m_LocalPlanner.m_TotalOriginalPath.at(0).size() > 3)
			{
				PlannerHNS::RelativeInfo info;
				bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_LocalPlanner.m_TotalOriginalPath, m_LocalPlanner.state, 0.75, info);
				if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < (int)m_LocalPlanner.m_TotalOriginalPath.size() && info.iFront > 0 && info.iFront < (int)m_LocalPlanner.m_TotalOriginalPath.at(info.iGlobalPath).size())
				{
					PlannerHNS::WayPoint wp_end = m_LocalPlanner.m_TotalOriginalPath.at(info.iGlobalPath).at(m_LocalPlanner.m_TotalOriginalPath.at(info.iGlobalPath).size()-1);
					PlannerHNS::WayPoint wp_first = m_LocalPlanner.m_TotalOriginalPath.at(info.iGlobalPath).at(info.iFront);
					double remaining_distance =   hypot(wp_end.pos.y - wp_first.pos.y, wp_end.pos.x - wp_first.pos.x) + info.to_front_distance;
					if(remaining_distance <= REPLANNING_DISTANCE)
					{

						cout << "Remaining Distance : " << remaining_distance << endl;
						bMakeNewPlan = true;
						if(m_SimParams.bLooper)
							InitializeSimuCar(m_SimParams.startPose);
					}
				}
			}
			else
				bMakeNewPlan = true;

			if(bMakeNewPlan)
			{
				std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths;
				vector<int> globalPathIds;
				if(m_SimParams.bRandomGoal)
					m_GlobalPlanner.PlanUsingDPRandom(m_LocalPlanner.state, PLANNING_DISTANCE, m_Map, generatedTotalPaths);
				else
					m_GlobalPlanner.PlanUsingDP(m_LocalPlanner.state, m_SimParams.goalPose, 100000, false, globalPathIds, m_Map, generatedTotalPaths);

				for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), 0.75);
					PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.4, 0.25);
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
				}

				m_LocalPlanner.m_TotalOriginalPath = generatedTotalPaths;
				m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
			}

			if(bNewLightSignal)
			{
				m_PrevTrafficLight = m_CurrTrafficLight;
				bNewLightSignal = false;
			}
			//Local Planning
			m_TrackedClusters.clear();
			currBehavior = m_LocalPlanner.DoOneStep(dt, currStatus, m_TrackedClusters, 1, m_Map, 0, m_PrevTrafficLight, true);

			 //Odometry Simulation and Update
			m_LocalPlanner.SetSimulatedTargetOdometryReadings(desiredStatus.speed, desiredStatus.steer, desiredStatus.shift);
			m_LocalPlanner.UpdateState(desiredStatus, false);
			m_LocalPlanner.LocalizeMe(dt);
			currStatus.shift = desiredStatus.shift;
			currStatus.steer = m_LocalPlanner.m_CurrentSteering;
			currStatus.speed = m_LocalPlanner.m_CurrentVelocity;


			//Path Following and Control
			desiredStatus = m_PredControl.DoOneStep(dt, currBehavior, m_LocalPlanner.m_Path, m_LocalPlanner.state, currStatus, currBehavior.bNewPlan);

			displayFollowingInfo(m_LocalPlanner.m_TrajectoryCostsCalculatotor.m_SafetyBorder.points, m_LocalPlanner.state);
			visualizePath(m_LocalPlanner.m_Path);
			visualizeBehaviors();


			geometry_msgs::PoseArray sim_data;
			geometry_msgs::Pose p_id, p_pose, p_box;


			sim_data.header.frame_id = "map";
			sim_data.header.stamp = ros::Time();

			p_id.position.x = m_SimParams.id;

			PlannerHNS::WayPoint pose_center = GetRealCenter(m_LocalPlanner.state);

			p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(pose_center.pos.a));
			p_pose.position.x = pose_center.pos.x;
			p_pose.position.y = pose_center.pos.y;
			p_pose.position.z = pose_center.pos.z;

			p_box.position.x = m_CarInfo.width;
			p_box.position.y = m_CarInfo.length;
			p_box.position.z = 2.2;

			sim_data.poses.push_back(p_id);
			sim_data.poses.push_back(p_pose);
			sim_data.poses.push_back(p_box);

			pub_SimuBoxPose.publish(sim_data);

			if(currBehavior.bNewPlan && m_SimParams.bEnableLogs)
			{
				std::ostringstream str_out;
				str_out << m_SimParams.logPath;
				str_out << "LocalPath_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(),  m_LocalPlanner.m_Path);
			}

			if(m_SimParams.bLooper && currBehavior.state == PlannerHNS::FINISH_STATE)
			{
				InitializeSimuCar(m_SimParams.startPose);
			}
		}

		loop_rate.sleep();
	}
}

}
