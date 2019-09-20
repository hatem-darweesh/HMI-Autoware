/*
 * BehaviorPrediction.cpp
 *
 *  Created on: Jul 6, 2017
 *      Author: user
 */

#include "BehaviorPrediction.h"
#include "MappingHelpers.h"



namespace PlannerHNS
{

BehaviorPrediction::BehaviorPrediction()
{

}

BehaviorPrediction::~BehaviorPrediction()
{

}

void BehaviorPrediction::DoOneStep(const std::vector<DetectedObject>& obj_list, RoadNetwork& map)
{
	FilterObservations(obj_list, map, m_TrackedObjects);
	//std::cout << "Before Filter:" << obj_list.size() << ", After: " <<  m_TrackedObjects.size() << std::endl;
	PredictionStep(m_TrackedObjects, m_ParticleInfo);
	//CorrectionStep(m_TrackedObjects, m_ParticleInfo);
	CorrectionStep2(m_ParticleInfo);
}

void BehaviorPrediction::FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list)
{
	std::vector<DetectedObject> temp_list;
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		if(obj_list.at(i).t == SIDEWALK)
			continue;

		WayPoint* pWP =  MappingHelpers::GetClosestWaypointFromMap(obj_list.at(i).center, map, obj_list.at(i).bDirection);
		if(pWP == 0 ) //|| hypot(pWP->pos.y - obj_list.at(i).center.pos.y, pWP->pos.x - obj_list.at(i).center.pos.x) > 1.5)
			continue;

		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < filtered_list.size(); ip++)
		{
			if(filtered_list.at(ip).id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(bFound)
		{
			double distance_diff =  hypot(obj_list.at(i).center.pos.y - filtered_list.at(found_index).center.pos.y, obj_list.at(i).center.pos.x - filtered_list.at(found_index).center.pos.x);
			double angle_diff = atan2(obj_list.at(i).center.pos.y -filtered_list.at(found_index).center.pos.y, obj_list.at(i).center.pos.x - filtered_list.at(found_index).center.pos.x);
			filtered_list.at(found_index) = obj_list.at(i);
			filtered_list.at(found_index).pClosestWaypoint = pWP;
			if(!filtered_list.at(found_index).bVelocity)
				filtered_list.at(found_index).center.v = distance_diff;
			if(!filtered_list.at(found_index).bDirection)
				filtered_list.at(found_index).center.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(angle_diff);
			temp_list.push_back(filtered_list.at(found_index));
		}
		else
		{
			filtered_list.push_back(obj_list.at(i));
			filtered_list.at(filtered_list.size()-1).pClosestWaypoint = pWP;
			temp_list.push_back(filtered_list.at(filtered_list.size()-1));
		}
	}

	filtered_list = temp_list;
}

void BehaviorPrediction::PredictionStep(std::vector<DetectedObject>& obj_list, std::vector<ObjParticles>& part_info)
{
	PlannerH planner;
	std::vector<ObjParticles> temp_list;
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < part_info.size(); ip++)
		{
			if(part_info.at(ip).obj.id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(bFound)
		{
			part_info.at(found_index).obj = obj_list.at(i);
			part_info.at(found_index).pred_paths.clear();
			planner.PredictPlanUsingDP(obj_list.at(i).center, obj_list.at(i).pClosestWaypoint, 10, part_info.at(found_index).pred_paths);
			//Prediction Step
			//PredictStopParticles(part_info.at(found_index));
			PredictStopParticles2(part_info.at(found_index));
			temp_list.push_back(part_info.at(found_index));
		}
		else
		{
			part_info.push_back(ObjParticles());
			part_info.at(part_info.size()-1).obj = obj_list.at(i);
			part_info.at(part_info.size()-1).pred_paths.clear();
			planner.PredictPlanUsingDP(obj_list.at(i).center, obj_list.at(i).pClosestWaypoint, 10, part_info.at(part_info.size()-1).pred_paths);

			//Initialize Particles
			GenerateParticles(part_info.at(part_info.size()-1));
			temp_list.push_back(part_info.at(part_info.size()-1));
		}
	}

	part_info = temp_list;
}

void BehaviorPrediction::GenerateParticles(ObjParticles& parts)
{
	Particle p;
	p.beh = STOPPING_STATE;
	p.vel = 0;
	p.acc = -1;
	p.indicator = 0;

	for(unsigned int i=0; i < STOP_PARTICLES_NUM; i++)
	{
		p.pose.x = parts.obj.center.pos.x;
		p.pose.y = parts.obj.center.pos.y;
		p.pose.z = parts.obj.center.pos.z;
		p.pose.a = parts.obj.center.pos.a;
		parts.particles.push_back(p);
		parts.particles_buff.push_back(p);
		parts.stop_particles.push_back(p);
	}

	p.beh = FORWARD_STATE;
	p.vel = 1;
	p.acc = 1;
	p.indicator = 0;

	for(unsigned int i=0; i < FORWARD_PARTICLES_NUM; i++)
	{
		p.pose.x = parts.obj.center.pos.x;
		p.pose.y = parts.obj.center.pos.y;
		p.pose.z = parts.obj.center.pos.z;
		p.pose.a = parts.obj.center.pos.a;
		parts.particles.push_back(p);
		parts.particles_buff.push_back(p);
		parts.forward_particles.push_back(p);
	}

	p.beh = BRANCH_RIGHT_STATE;
	p.vel = 1;
	p.acc = 1;
	p.indicator = 0;

	for(unsigned int i=0; i < RIGHT_BRANCH_PARTICLES_NUM; i++)
	{
		p.pose.x = parts.obj.center.pos.x;
		p.pose.y = parts.obj.center.pos.y;
		p.pose.z = parts.obj.center.pos.z;
		p.pose.a = parts.obj.center.pos.a;
		parts.particles.push_back(p);
		parts.particles_buff.push_back(p);
		parts.right_particles.push_back(p);
	}

	p.beh = BRANCH_LEFT_STATE;
	p.vel = 1;
	p.acc = 1;
	p.indicator = 0;

	for(unsigned int i=0; i < LEFT_BRANCH_PARTICLES_NUM; i++)
	{
		p.pose.x = parts.obj.center.pos.x;
		p.pose.y = parts.obj.center.pos.y;
		p.pose.z = parts.obj.center.pos.z;
		p.pose.a = parts.obj.center.pos.a;
		parts.particles.push_back(p);
		parts.particles_buff.push_back(p);
		parts.left_particles.push_back(p);
	}
}


void BehaviorPrediction::PredictStopParticles2(ObjParticles& parts)
{
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);

	ENG eng(t.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(0, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
	NormalDIST acl(0, MEASURE_ACL_ERROR);
	VariatGEN gen_acl(eng, acl);

	for(unsigned int i = 0; i < parts.stop_particles.size(); i++)
	{
		parts.stop_particles.at(i).beh = STOPPING_STATE;
		parts.stop_particles.at(i).pose.x = parts.obj.center.pos.x ;
		parts.stop_particles.at(i).pose.y = parts.obj.center.pos.y ;
		parts.stop_particles.at(i).pose.z = parts.obj.center.pos.z;
		parts.stop_particles.at(i).pose.a = parts.obj.center.pos.a ;
		parts.stop_particles.at(i).vel = 0;
		parts.stop_particles.at(i).indicator = 0;

	}

	WayPoint forward_wp, left_wp, right_wp;
	for(int i = 0; i < parts.pred_paths.size(); i++)
	{
		if(parts.pred_paths.at(i).size() > LOOK_AHEAD_INDEX+1)
		{
			if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_RIGHT_STATE)
				right_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_LEFT_STATE)
				left_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else
				forward_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
		}
	}

	for(unsigned int i = 0; i < parts.forward_particles.size(); i++)
	{
		parts.forward_particles.at(i).beh = FORWARD_STATE;
		parts.forward_particles.at(i).pose.x = forward_wp.pos.x + gen_x();
		parts.forward_particles.at(i).pose.y = forward_wp.pos.y + gen_x();
		parts.forward_particles.at(i).pose.a = forward_wp.pos.a + gen_a();
		parts.forward_particles.at(i).vel = parts.obj.center.v + gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.forward_particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.forward_particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.forward_particles.at(i).acc = 1;

		parts.forward_particles.at(i).indicator = 0;
	}

	for(unsigned int i = 0; i < parts.right_particles.size(); i++)
	{
		parts.right_particles.at(i).beh = FORWARD_STATE;
		parts.right_particles.at(i).pose.x = right_wp.pos.x + gen_x();
		parts.right_particles.at(i).pose.y = right_wp.pos.y + gen_x();
		parts.right_particles.at(i).pose.a = right_wp.pos.a + gen_a();
		parts.right_particles.at(i).vel = parts.obj.center.v +  gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.right_particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.right_particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.right_particles.at(i).acc = 1;

		parts.right_particles.at(i).indicator = 1;
	}

	for(unsigned int i = 0; i < parts.left_particles.size(); i++)
	{
		parts.left_particles.at(i).beh = FORWARD_STATE;
		parts.left_particles.at(i).pose.x = left_wp.pos.x + gen_x();
		parts.left_particles.at(i).pose.y = left_wp.pos.y + gen_x();
		parts.left_particles.at(i).pose.a = left_wp.pos.a + gen_a();
		parts.left_particles.at(i).vel = parts.obj.center.v +  gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.left_particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.left_particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.left_particles.at(i).acc = 1;

		parts.left_particles.at(i).indicator = 1;
	}

}

void BehaviorPrediction::PredictStopParticles(ObjParticles& parts)
{
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);

	ENG eng(t.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
	NormalDIST acl(0, MEASURE_ACL_ERROR);
	VariatGEN gen_acl(eng, acl);


	for(int i = 0; i < STOP_PARTICLES_NUM; i++)
	{
//		if(parts.obj.bVelocity)
//		{
//			parts.particles.at(i).pose.x = parts.particles.at(i).pose.x + ((parts.obj.center.v) * cos(parts.particles.at(i).pose.a)*MOTION_DT) + gen_d();
//			parts.particles.at(i).pose.y = parts.particles.at(i).pose.y + ((parts.obj.center.v) * sin(parts.particles.at(i).pose.a)*MOTION_DT) + gen_d();
//		}
//		else
//		{
//			parts.particles.at(i).pose.x = parts.particles.at(i).pose.x + ((parts.obj.center.v) * cos(parts.particles.at(i).pose.a)) + gen_d();
//			parts.particles.at(i).pose.y = parts.particles.at(i).pose.y + ((parts.obj.center.v) * sin(parts.particles.at(i).pose.a)) + gen_d();
//		}
		parts.particles.at(i).beh = STOPPING_STATE;
		parts.particles.at(i).pose.x = parts.obj.center.pos.x + gen_x();
		parts.particles.at(i).pose.y = parts.obj.center.pos.y + gen_x();
		parts.particles.at(i).pose.z = parts.obj.center.pos.z;
		parts.particles.at(i).pose.a = parts.obj.center.pos.a + gen_a();
		parts.particles.at(i).vel = gen_v();
		parts.particles.at(i).indicator = 0;

	}

	WayPoint forward_wp, left_wp, right_wp;
	for(int i = 0; i < parts.pred_paths.size(); i++)
	{
		if(parts.pred_paths.at(i).size() > LOOK_AHEAD_INDEX+1)
		{
			if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_RIGHT_STATE)
				right_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_LEFT_STATE)
				left_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else
				forward_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
		}
	}

	int next_inc = STOP_PARTICLES_NUM -1;
	//int next_inc = 0;

	for(int i=next_inc; i < next_inc+FORWARD_PARTICLES_NUM; i++)
	{
		parts.particles.at(i).beh = FORWARD_STATE;
		parts.particles.at(i).pose.x = forward_wp.pos.x + gen_x();
		parts.particles.at(i).pose.y = forward_wp.pos.y + gen_x();
		parts.particles.at(i).pose.a = forward_wp.pos.a + gen_a();
		parts.particles.at(i).vel = parts.obj.center.v +  gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.particles.at(i).acc = 1;

		parts.particles.at(i).indicator = 0;
	}

	next_inc += FORWARD_PARTICLES_NUM -1;

	for(int i=next_inc; i < RIGHT_BRANCH_PARTICLES_NUM+next_inc; i++)
	{
		parts.particles.at(i).beh = BRANCH_RIGHT_STATE;
		parts.particles.at(i).pose.x = right_wp.pos.x + gen_x();
		parts.particles.at(i).pose.y = right_wp.pos.y + gen_x();
		parts.particles.at(i).pose.a = right_wp.pos.a + gen_a();

		parts.particles.at(i).vel =  parts.obj.center.v + gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.particles.at(i).acc = 1;

		parts.particles.at(i).indicator = 1;
	}

	next_inc += RIGHT_BRANCH_PARTICLES_NUM -1;

	for(int i=next_inc; i < LEFT_BRANCH_PARTICLES_NUM + next_inc; i++)
	{
		parts.particles.at(i).beh = BRANCH_LEFT_STATE;
		parts.particles.at(i).pose.x = left_wp.pos.x + gen_x();
		parts.particles.at(i).pose.y = left_wp.pos.y + gen_x();
		parts.particles.at(i).pose.a = left_wp.pos.a + gen_a();

		parts.particles.at(i).vel = parts.obj.center.v + gen_v();

		double acl_err = gen_acl();
		if(acl_err >= -0.25 && acl_err <= 0.25)
			parts.particles.at(i).acc = 0;
		else if(acl_err < -0.25)
			parts.particles.at(i).acc = -1;
		else if(acl_err > 0.25)
			parts.particles.at(i).acc = 1;

		parts.particles.at(i).indicator = -1;
	}
}

void BehaviorPrediction::CorrectStopParticles(ObjParticles& parts)
{
	parts.all_w = 0;

	for(int i=0; i < STOP_PARTICLES_NUM; i++)
	{

		parts.particles.at(i).w = exp(-(
//				pow(parts.particles.at(i).pose.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//				pow(parts.particles.at(i).pose.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
				pow(parts.particles.at(i).pose.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)+
				pow(parts.particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

		parts.all_w += parts.particles.at(i).w;
	}

	WayPoint forward_wp, left_wp, right_wp;
	for(int i = 0; i < parts.pred_paths.size(); i++)
	{
		if(parts.pred_paths.at(i).size() > LOOK_AHEAD_INDEX+1)
		{
			if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_RIGHT_STATE)
				right_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else if(parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX).behavior == BRANCH_LEFT_STATE)
				left_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
			else
				forward_wp = parts.pred_paths.at(i).at(LOOK_AHEAD_INDEX);
		}
	}

	int next_inc = STOP_PARTICLES_NUM -1;
	//int next_inc = 0;

	for(int i=next_inc; i < next_inc+FORWARD_PARTICLES_NUM; i++)
	{
		parts.particles.at(i).w = exp(-(
//			pow(parts.particles.at(i).pose.x - forward_wp.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//			pow(parts.particles.at(i).pose.y - forward_wp.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR) +
			pow(parts.particles.at(i).pose.a - forward_wp.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR) +
			pow(parts.particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

		parts.all_w += parts.particles.at(i).w;
	}

	next_inc += FORWARD_PARTICLES_NUM -1;

	for(int i=next_inc; i < RIGHT_BRANCH_PARTICLES_NUM+next_inc; i++)
	{
		parts.particles.at(i).w = exp(-(
//			pow(parts.particles.at(i).pose.x - right_wp.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//			pow(parts.particles.at(i).pose.y - right_wp.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR) +
			pow(parts.particles.at(i).pose.a - right_wp.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR) +
			pow(parts.particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

		parts.all_w += parts.particles.at(i).w;
	}

	next_inc += RIGHT_BRANCH_PARTICLES_NUM -1;

	for(int i=next_inc; i < LEFT_BRANCH_PARTICLES_NUM + next_inc; i++)
	{
		parts.particles.at(i).w = exp(-(
//			pow(parts.particles.at(i).pose.x - left_wp.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//			pow(parts.particles.at(i).pose.y - left_wp.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR) +
			pow(parts.particles.at(i).pose.a - left_wp.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR) +
			pow(parts.particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

		parts.all_w += parts.particles.at(i).w;
	}


}

void BehaviorPrediction::CorrectStopParticles2(ObjParticles& parts)
{
	parts.all_w = 0;
	parts.pose_w_t = 0;
	parts.dir_w_t = 0;
	parts.vel_w_t = 0;

	parts.pose_w_max = -99999999;
	parts.dir_w_max = -99999999;
	parts.vel_w_max = -99999999;

	parts.pose_w_min = 99999999;
	parts.dir_w_min = 99999999;
	parts.vel_w_min = 99999999;


//	for(unsigned int i=0; i < parts.stop_particles.size(); i++)
//	{
//		parts.stop_particles.at(i).pose_w =  0.5*(pow(parts.stop_particles.at(i).pose.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//					pow(parts.stop_particles.at(i).pose.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR));
//		parts.stop_particles.at(i).dir_w = pow(parts.stop_particles.at(i).pose.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR);
//		parts.stop_particles.at(i).vel_w =  pow(parts.stop_particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR);
//
//		parts.pose_w_t += parts.stop_particles.at(i).pose_w;
//		parts.dir_w_t += parts.stop_particles.at(i).dir_w;
//		parts.vel_w_t += parts.stop_particles.at(i).vel_w;
//
//		if(parts.stop_particles.at(i).pose_w > parts.pose_w_max)
//			parts.pose_w_max = parts.stop_particles.at(i).pose_w;
//
//		if(parts.stop_particles.at(i).dir_w > parts.dir_w_max)
//			parts.dir_w_max = parts.stop_particles.at(i).dir_w;
//
//		if(parts.stop_particles.at(i).vel_w > parts.vel_w_max)
//			parts.vel_w_max = parts.stop_particles.at(i).vel_w;
//
//		if(parts.stop_particles.at(i).pose_w < parts.pose_w_min)
//			parts.pose_w_min = parts.stop_particles.at(i).pose_w;
//
//		if(parts.stop_particles.at(i).dir_w < parts.dir_w_min)
//			parts.dir_w_min = parts.stop_particles.at(i).dir_w;
//
//		if(parts.stop_particles.at(i).vel_w < parts.vel_w_min)
//			parts.vel_w_min = parts.stop_particles.at(i).vel_w;
//
//	}

	for(unsigned int i=0; i < parts.forward_particles.size(); i++)
	{
		parts.forward_particles.at(i).pose_w =  0.5*(pow(parts.forward_particles.at(i).pose.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
					pow(parts.forward_particles.at(i).pose.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR));
		parts.forward_particles.at(i).dir_w = pow(parts.forward_particles.at(i).pose.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR);
		parts.forward_particles.at(i).vel_w =  pow(parts.forward_particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR);

		parts.pose_w_t += parts.forward_particles.at(i).pose_w;
		parts.dir_w_t += parts.forward_particles.at(i).dir_w;
		parts.vel_w_t += parts.forward_particles.at(i).vel_w;

		if(parts.forward_particles.at(i).pose_w > parts.pose_w_max)
			parts.pose_w_max = parts.forward_particles.at(i).pose_w;

		if(parts.forward_particles.at(i).dir_w > parts.dir_w_max)
			parts.dir_w_max = parts.forward_particles.at(i).dir_w;

		if(parts.forward_particles.at(i).vel_w > parts.vel_w_max)
			parts.vel_w_max = parts.forward_particles.at(i).vel_w;

		if(parts.forward_particles.at(i).pose_w < parts.pose_w_min)
			parts.pose_w_min = parts.forward_particles.at(i).pose_w;

		if(parts.forward_particles.at(i).dir_w < parts.dir_w_min)
			parts.dir_w_min = parts.forward_particles.at(i).dir_w;

		if(parts.forward_particles.at(i).vel_w < parts.vel_w_min)
			parts.vel_w_min = parts.forward_particles.at(i).vel_w;

	}
//
//	for(unsigned int i=0; i < parts.right_particles.size(); i++)
//	{
//		parts.right_particles.at(i).pose_w =  0.5*(pow(parts.right_particles.at(i).pose.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//					pow(parts.right_particles.at(i).pose.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR));
//		parts.right_particles.at(i).dir_w = pow(parts.right_particles.at(i).pose.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR);
//		parts.right_particles.at(i).vel_w =  pow(parts.right_particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR);
//
//		parts.pose_w_t += parts.right_particles.at(i).pose_w;
//		parts.dir_w_t += parts.right_particles.at(i).dir_w;
//		parts.vel_w_t += parts.right_particles.at(i).vel_w;
//
//		if(parts.right_particles.at(i).pose_w > parts.pose_w_max)
//			parts.pose_w_max = parts.right_particles.at(i).pose_w;
//
//		if(parts.right_particles.at(i).dir_w > parts.dir_w_max)
//			parts.dir_w_max = parts.right_particles.at(i).dir_w;
//
//		if(parts.right_particles.at(i).vel_w > parts.vel_w_max)
//			parts.vel_w_max = parts.right_particles.at(i).vel_w;
//
//		if(parts.right_particles.at(i).pose_w < parts.pose_w_min)
//			parts.pose_w_min = parts.right_particles.at(i).pose_w;
//
//		if(parts.right_particles.at(i).dir_w < parts.dir_w_min)
//			parts.dir_w_min = parts.right_particles.at(i).dir_w;
//
//		if(parts.right_particles.at(i).vel_w < parts.vel_w_min)
//			parts.vel_w_min = parts.right_particles.at(i).vel_w;
//
//	}
//
//	for(unsigned int i=0; i < parts.left_particles.size(); i++)
//	{
//		parts.left_particles.at(i).pose_w =  0.5*(pow(parts.left_particles.at(i).pose.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+
//					pow(parts.left_particles.at(i).pose.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR));
//		parts.left_particles.at(i).dir_w = pow(parts.left_particles.at(i).pose.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR);
//		parts.left_particles.at(i).vel_w =  pow(parts.left_particles.at(i).vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR);
//
//		parts.pose_w_t += parts.left_particles.at(i).pose_w;
//		parts.dir_w_t += parts.left_particles.at(i).dir_w;
//		parts.vel_w_t += parts.left_particles.at(i).vel_w;
//
//		if(parts.left_particles.at(i).pose_w > parts.pose_w_max)
//			parts.pose_w_max = parts.left_particles.at(i).pose_w;
//
//		if(parts.left_particles.at(i).dir_w > parts.dir_w_max)
//			parts.dir_w_max = parts.left_particles.at(i).dir_w;
//
//		if(parts.left_particles.at(i).vel_w > parts.vel_w_max)
//			parts.vel_w_max = parts.left_particles.at(i).vel_w;
//
//		if(parts.left_particles.at(i).pose_w < parts.pose_w_min)
//			parts.pose_w_min = parts.left_particles.at(i).pose_w;
//
//		if(parts.left_particles.at(i).dir_w < parts.dir_w_min)
//			parts.dir_w_min = parts.left_particles.at(i).dir_w;
//
//		if(parts.left_particles.at(i).vel_w < parts.vel_w_min)
//			parts.vel_w_min = parts.left_particles.at(i).vel_w;
//
//	}

}

void BehaviorPrediction::MakeYieldParticles(ObjParticles& parts)
{

}

void BehaviorPrediction::MakeForwardParticles(ObjParticles& parts)
{

}

void BehaviorPrediction::MakeBranchParticles(ObjParticles& parts)
{

}

void BehaviorPrediction::CorrectionStep2(std::vector<ObjParticles>& part_info)
{

	for(unsigned int i=0; i < part_info.size(); i++)
	{
		part_info.at(i).max_w = -9999999;
		part_info.at(i).min_w = 9999999;
		part_info.at(i).all_w = 0;
		double stop_sum = 0;
		double forward_sum = 0;

		CorrectStopParticles2(part_info.at(i));
		double pose_diff  = part_info.at(i).pose_w_max-part_info.at(i).pose_w_min;
		double dir_diff = part_info.at(i).dir_w_max-part_info.at(i).dir_w_min;
		double vel_diff = part_info.at(i).vel_w_max-part_info.at(i).vel_w_min;

//		for(unsigned int ip=0; ip < part_info.at(i).stop_particles.size(); ip++)
//		{
//			if(pose_diff != 0)
//				part_info.at(i).stop_particles.at(ip).pose_w = part_info.at(i).stop_particles.at(ip).pose_w/pose_diff;
//			else
//				part_info.at(i).stop_particles.at(ip).pose_w = 0;
//
//			if(dir_diff != 0)
//				part_info.at(i).stop_particles.at(ip).dir_w = part_info.at(i).stop_particles.at(ip).dir_w/dir_diff;
//			else
//				part_info.at(i).stop_particles.at(ip).dir_w = 0;
//
//			if(vel_diff != 0 )
//				part_info.at(i).stop_particles.at(ip).vel_w = part_info.at(i).stop_particles.at(ip).vel_w/vel_diff;
//			else
//				part_info.at(i).stop_particles.at(ip).vel_w = 0;
//
//			part_info.at(i).stop_particles.at(ip).w = (part_info.at(i).stop_particles.at(ip).pose_w + part_info.at(i).stop_particles.at(ip).dir_w + part_info.at(i).stop_particles.at(ip).vel_w)/3.0;
//
//			if(part_info.at(i).stop_particles.at(ip).w >= part_info.at(i).max_w)
//				part_info.at(i).max_w = part_info.at(i).stop_particles.at(ip).w;
//
//			if(part_info.at(i).stop_particles.at(ip).w <= part_info.at(i).min_w)
//				part_info.at(i).min_w = part_info.at(i).stop_particles.at(ip).w;
//
//			stop_sum += part_info.at(i).stop_particles.at(ip).w;
//		}

		for(unsigned int ip=0; ip < part_info.at(i).forward_particles.size(); ip++)
		{
			if(pose_diff != 0)
				part_info.at(i).forward_particles.at(ip).pose_w = part_info.at(i).forward_particles.at(ip).pose_w/pose_diff;
			else
				part_info.at(i).forward_particles.at(ip).pose_w = 0;

			if(dir_diff != 0)
				part_info.at(i).forward_particles.at(ip).dir_w = part_info.at(i).forward_particles.at(ip).dir_w/dir_diff;
			else
				part_info.at(i).forward_particles.at(ip).dir_w = 0;

			if(vel_diff != 0 )
				part_info.at(i).forward_particles.at(ip).vel_w = part_info.at(i).forward_particles.at(ip).vel_w/vel_diff;
			else
				part_info.at(i).forward_particles.at(ip).vel_w = 0;

			part_info.at(i).forward_particles.at(ip).w = (part_info.at(i).forward_particles.at(ip).pose_w + part_info.at(i).forward_particles.at(ip).dir_w + part_info.at(i).forward_particles.at(ip).vel_w)/3.0;

			if(part_info.at(i).forward_particles.at(ip).w >= part_info.at(i).max_w)
				part_info.at(i).max_w = part_info.at(i).forward_particles.at(ip).w;

			if(part_info.at(i).forward_particles.at(ip).w <= part_info.at(i).min_w)
				part_info.at(i).min_w = part_info.at(i).forward_particles.at(ip).w;

			forward_sum += part_info.at(i).forward_particles.at(ip).vel_w;
		}

		double stop_avg = stop_sum / part_info.at(i).stop_particles.size();
		double forward_avg = forward_sum / part_info.at(i).forward_particles.size();


		if(stop_avg < forward_avg)
			std::cout << "(" << part_info.at(i).obj.id << " is Stopping ) Stop = " << stop_avg << ", Forward = " << forward_avg <<  std::endl;
		else
			std::cout << "(" << part_info.at(i).obj.id << " is Moving ) Stop = " << stop_avg << ", Forward = " << forward_avg <<  std::endl;

		//std::cout << "(" << part_info.at(i).obj.id << ") Pose(" << part_info.at(i).max_w  <<  ", "<< part_info.at(i).min_w << ", " << part_info.at(i).avg_w << ")" <<  std::endl;

//		std::cout << "(" << part_info.at(i).obj.id << ") Pose(" << part_info.at(i).pose_w_max  <<  ", "<< part_info.at(i).pose_w_min << ", " << part_info.at(i).pose_w_t << ")"<< std::endl;
//		std::cout << "(" << part_info.at(i).obj.id << ") Dire(" << part_info.at(i).dir_w_max  <<  ", "<< part_info.at(i).dir_w_min << ", " << part_info.at(i).dir_w_t << ")"<< std::endl;
//		std::cout << "(" << part_info.at(i).obj.id << ") Velo(" << part_info.at(i).vel_w_max  <<  ", "<< part_info.at(i).vel_w_min << ", " << part_info.at(i).vel_w_t << ")"<< std::endl;
	}


}


void BehaviorPrediction::CorrectionStep(std::vector<DetectedObject>& obj_list, std::vector<ObjParticles>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		part_info.at(i).max_w = -100;
		double sum = 0;
		part_info.at(i).min_w = 100;
		//std::cout << "Weights For Obj(" << part_info.at(i).obj.id << ") " << std::endl;
		CorrectStopParticles(part_info.at(i));

		double r = (rand()/(double)RAND_MAX)/(double)part_info.at(i).particles.size();
		int index = 0;
		double  c = (part_info.at(i).particles.at(index).w/part_info.at(i).all_w);
		for(unsigned int ip=0; ip < part_info.at(i).particles.size(); ip++)
		{
			//std::cout << "(" << ip << ") => " << part_info.at(i).particles.at(index).w << std::endl;
			double U = r + ip /(double)part_info.at(i).particles.size();
			while(U > c)
			{
			  index++;
			  if(index >= part_info.at(i).particles.size())
				  index = 0;

			  c += (part_info.at(i).particles.at(index).w/part_info.at(i).all_w);
			}

			//if(part_info.at(i).particles.at(index).beh == part_info.at(i).particles_buff.at(ip).beh)
				part_info.at(i).particles_buff.at(ip) = part_info.at(i).particles.at(index);

			sum += part_info.at(i).particles.at(ip).w;
			if(part_info.at(i).particles.at(ip).w > part_info.at(i).max_w)
				part_info.at(i).max_w = part_info.at(i).particles.at(ip).w;

			if(part_info.at(i).particles.at(ip).w < part_info.at(i).min_w)
				part_info.at(i).min_w = part_info.at(i).particles.at(ip).w;
		}

		part_info.at(i).avg_w = sum/(double)part_info.at(i).particles.size();

//		std::cout << "Max of (" << part_info.at(i).obj.id << ") = " << part_info.at(i).max_w
//				<<  ", Avg = "<< sum/(double)part_info.at(i).particles.size()
//				<<  ", Min = "<< part_info.at(i).min_w
//				<< std::endl;
		//std::cout << std::endl << std::endl;
		part_info.at(i).particles = part_info.at(i).particles_buff;
	}
}

double BehaviorPrediction::nrand(double n)
{
  double r;
  r = n*sqrt(-2.0*log((double)rand()/RAND_MAX))*cos(2.0*M_PI*rand()/RAND_MAX);
  return r;

}

} /* namespace PlannerHNS */
