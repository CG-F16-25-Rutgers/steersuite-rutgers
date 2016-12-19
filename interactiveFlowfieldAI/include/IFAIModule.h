// Interactive Flowfield - by Qi Dong 

#ifndef __IFAI_MODULE__
#define __IFAI_MODULE__

#include "SteerLib.h"
#include <vector>
#include "Logger.h"

struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};

class IFAIModule : public SteerLib::ModuleInterface
{
public:
	
	void cleanupSimulation();
	void preprocessSimulation();
	void initializeSimulation();
	//std::string getDependencies() { return "testCasePlayer"; }
	std::string getDependencies() { return ""; }
	
	std::string getConflicts() { return ""; }
	std::string getData() { return _data; }
	LogData * getLogData()
	{
		LogData * lD = new LogData();
		lD->setLogger(this->_rvoLogger);
		lD->setLogData(this->_logData);
		return lD;
	}
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	std::vector<SteerLib::AgentInterface * > agents;
	
	unsigned int gLongTermPlanningPhaseInterval;
	unsigned int gMidTermPlanningPhaseInterval;
	unsigned int gShortTermPlanningPhaseInterval;
	unsigned int gPredictivePhaseInterval;
	unsigned int gReactivePhaseInterval;
	unsigned int gPerceptivePhaseInterval;
	bool gUseDynamicPhaseScheduling;
	bool gShowStats;
	bool gShowAllStats;
	
	PhaseProfilers * gPhaseProfilers;
	
	float sf_acceleration;
	float sf_personal_space_threshold;
	float sf_agent_repulsion_importance;
	float sf_query_radius;
	float sf_body_force;
	float sf_agent_body_force;
	float sf_sliding_friction_force;
	float sf_agent_b;
	float sf_agent_a;
	float sf_wall_b;
	float sf_wall_a;
	float sf_max_speed;
	bool dont_plan=false;
	//float sf_preferred_speed;
	float maxWeight;
	bool useLeader;//every goal can only have a single leader

protected:
	std::string logFilename; // = "pprAI.log";
	bool logStats; // = false;
	Logger * _rvoLogger;
	std::string _data;
	std::vector<LogObject *> _logData;

	SteerLib::EngineInterface * _gEngine;
	SteerLib::EngineInterface * gEngine;
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
};

#endif