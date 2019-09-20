/*
 * BehaviorPrediction.h
 *
 *  Created on: Jul 6, 2017
 *      Author: user
 */

#ifndef BEHAVIORPREDICTION_H_
#define BEHAVIORPREDICTION_H_

#include "PlannerH.h"
#include "UtilityH.h"
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

#define MOTION_POSE_ERROR 0.25 // 50 cm pose error
#define MOTION_ANGLE_ERROR 0.05 // 0.05 rad angle error
#define MOTION_VEL_ERROR 1
#define MEASURE_POSE_ERROR 0.25
#define MEASURE_ANGLE_ERROR 0.05
#define MEASURE_VEL_ERROR 1
#define MEASURE_ACL_ERROR 1

#define STOP_PARTICLES_NUM 150
#define FORWARD_PARTICLES_NUM 150
#define YIELD_PARTICLES_NUM 50
#define RIGHT_BRANCH_PARTICLES_NUM 50
#define LEFT_BRANCH_PARTICLES_NUM 50
#define MOTION_DT 0.01
#define LOOK_AHEAD_INDEX 3



namespace PlannerHNS
{

typedef boost::mt19937 ENG;
typedef boost::normal_distribution<double> NormalDIST;
typedef boost::variate_generator<ENG, NormalDIST> VariatGEN;

class Particle
{
public:
	STATE_TYPE beh; //[Stop, Yielding, Forward, Branching]
	int vel; //[0 -> Stop,1 -> moving]
	int acc; //[-1 ->Slowing, 1 -> accelerating]
	int indicator; //[-1 -> Left, 0 -> no, 1 -> Right ]
	GPSPoint pose;
	double w;
	double pose_w;
	double dir_w;
	double vel_w;
	double acl_w;
	double ind_w;

	Particle()
	{
		w = 0;
		pose_w = 0;
		dir_w = 0;
		vel_w = 0;
		acl_w = 0;
		ind_w = 0;
		beh = INITIAL_STATE;
		vel = 0;
		acc = 0;
		indicator = 0;
	}
};

class ObjParticles
{
public:
	DetectedObject obj;
	std::vector<Particle> particles;
	std::vector<Particle> particles_buff;

	std::vector<Particle> stop_particles;
	std::vector<Particle> forward_particles;
	std::vector<Particle> right_particles;
	std::vector<Particle> left_particles;

	std::vector<std::vector<WayPoint> > pred_paths;
	double all_w;
	double max_w;
	double min_w;
	double avg_w;
	double pose_w_t;
	double dir_w_t;
	double vel_w_t;
	double acl_w_t;
	double ind_w_t;

	double pose_w_max;
	double dir_w_max;
	double vel_w_max;
	double acl_w_max;
	double ind_w_max;

	double pose_w_min;
	double dir_w_min;
	double vel_w_min;
	double acl_w_min;
	double ind_w_min;


	ObjParticles()
	{
		all_w = 0;
		pose_w_t = 0;
		dir_w_t = 0;
		vel_w_t = 0;
		acl_w_t = 0;
		ind_w_t = 0;
		max_w = -100;
		min_w = 100;
		avg_w = 0;

		pose_w_max = -999999;
		dir_w_max=-999999;
		vel_w_max=-999999;
		acl_w_max=-999999;
		ind_w_max=-999999;

		pose_w_min=999999;
		dir_w_min=999999;
		vel_w_min=999999;
		acl_w_min=999999;
		ind_w_min=999999;
	}
};

class BehaviorPrediction
{
public:
	BehaviorPrediction();
	virtual ~BehaviorPrediction();
	void DoOneStep(const std::vector<DetectedObject>& obj_list, RoadNetwork& map);

public:
	std::vector<ObjParticles> m_ParticleInfo;
protected:
	void FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list);
	void PredictionStep(std::vector<DetectedObject>& obj_list, std::vector<ObjParticles>& part_info);
	void CorrectionStep(std::vector<DetectedObject>& obj_list, std::vector<ObjParticles>& part_info);
	void CorrectionStep2(std::vector<ObjParticles>& part_info);

	void GenerateParticles(ObjParticles& parts);

	void PredictStopParticles(ObjParticles& parts);
	void CorrectStopParticles(ObjParticles& parts);

	void PredictStopParticles2(ObjParticles& parts);
	void CorrectStopParticles2(ObjParticles& parts);

	void MakeYieldParticles(ObjParticles& parts);
	void MakeForwardParticles(ObjParticles& parts);
	void MakeBranchParticles(ObjParticles& parts);

	std::vector<DetectedObject> m_TrackedObjects;

private:
	double nrand(double n);
};



} /* namespace PlannerHNS */

#endif /* BEHAVIORPREDICTION_H_ */
