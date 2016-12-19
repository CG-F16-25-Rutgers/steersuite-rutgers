

#ifndef __IFAI_Agent__
#define __IFAI_Agent__

#include <queue>
#include <list>
#include "SteerLib.h"
#include "IFAIModule.h"
#include <vector>
#include <stack>
#include <set>
#include <map>
#include <unordered_map>

// #define DRAW_ANNOTATIONS 1
// #define DRAW_HISTORIES 1
// #define DRAW_COLLISIONS 1
#define AGENT_MASS 1.0f



#define MAX_SPEED 2.6f
#define PERFERED_SPEED 2.33 // TODO not added to parameters yet.


#define ACCELERATION 0.2 // = v/A
#define PERSONAL_SPACE_THRESHOLD 0.2 // not defined in HiDAC papaer
#define AGENT_REPULSION_IMPORTANCE 0.51 // in HiDAC
#define QUERY_RADIUS 4.0f // not defined in paper
#define BODY_FORCE 2100.0f // K (big K) 120000 / 80
#define AGENT_BODY_FORCE 2900.0f
#define SLIDING_FRICTION_FORCE 2400.0f // k (small k) 240000 / 80 = 3000
#define AGENT_B 0.12f // inverse proximity force importance
#define AGENT_A 40.0f // 2000 / 80 Yep its just called A... inverse proximity force importance
#define WALL_B 0.12f //  inverse proximity force importance
#define WALL_A 45.0f //  proximity force importance
#define FURTHEST_LOCAL_TARGET_DISTANCE 45


#define MASS 1
// #define WAYPOINT_THRESHOLD_MULTIPLIER 2.5
// #define GOAL_THRESHOLD_MULTIPLIER 10.5
#define WAYPOINT_THRESHOLD_MULTIPLIER 1
#define GOAL_THRESHOLD_MULTIPLIER 2.5

#define USE_PLANNING 1
// #define DRAW_ANNOTATIONS 1
#define USE_CIRCLES 1
// #define _DEBUG_ 1


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define MAXWEIGHT 70


enum IFNodeStatus{
	UNDEFINED,
	OPEN,
	CLOSED,
	INCONSISTENT
};
class IFNode{
	//a node for use within a goal field
public:
	static SteerLib::SpatialDataBaseInterface* gSpatialDatabase;
	int id=-10000;Util::Point point;
	double distance=DBL_MAX;
	IFNode* next=NULL;
	IFNodeStatus status=UNDEFINED;
	std::vector<IFNode *>* neighbors=NULL;//neighbors are set when the node is about to be expanded, not when it is created
	bool _isClear=false;
	bool isClear(int ID=-10000)
	{
		if(ID!=-10000)
		{
			if(id==-10000)
			{
				id=ID;gSpatialDatabase->getLocationFromIndex(id, point);
				_isClear=(gSpatialDatabase->getTraversalCost(id)<1000);
			}
			else {if(id!=ID){std::cout << "repeated initialization of node! " <<id<< " as "<<ID<<std::endl;return false;}}
		}
		return _isClear;
	}
};

class IFAgent;
class IFGoal{
	// a goal with precomputed best paths from all places ever queried
public:
	Util::Point goal;int goalID;
	IFAgent* leader=NULL; Util::Point leaderStart; double leaderDistance=DBL_MAX;
	static SteerLib::SpatialDataBaseInterface* gSpatialDatabase;
	std::vector<IFNode *> closedSet; 
	std::vector<IFNode *> openSet;
	std::unordered_map<int,IFNode> nodes;
	Util::Point start;
	IFGoal(int id)
	{
			goalID=id;gSpatialDatabase->getLocationFromIndex(id, goal);
			nodes[goalID].isClear(id);//silently initialize it
			nodes[goalID].distance=0;//forgot this... LOL
			insertOpen(&nodes[goalID]);
	}
	~IFGoal()
	{
	}
	Util::Point getPointFromGridIndex(int id)  
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}
	IFNode* getNodeFromPoint(Util::Point p)
	{
		int id=gSpatialDatabase->getCellIndexFromLocation(p.x,p.z);
		nodes[id].isClear(id);
		return &(nodes[id]);
	}
	bool prepareField(Util::Point start,float margin=0)
	{
		//flood-calculate distance field until all open nodes are at least distance to start + margin away from the goal so this field can safely serve as heuristic for a forward search from start to goal, when the optimistic path from start to goal has extra dynamic cost(from congestion etc) = margin
		//if the optimistic path to start is unknown, first calculate it
		this->start=start;double startDistance=-1;
		int startID=gSpatialDatabase->getCellIndexFromLocation(start.x,start.z);
		int goalID=gSpatialDatabase->getCellIndexFromLocation(goal.x,goal.z);
		
		
		if(getNodeFromPoint(start)->status!=CLOSED)
		{
			IFNode * currentNode;
			while((openSet.size()>0))
			{//the open set is already sorted by distance
				currentNode = popOpen();
				insertClosed(currentNode);//should stop after the target's neighbors are updated! or later node paths may be invalid, as the current target can be a node on another path
				std::vector<IFNode*>* neighborNodes=getNeighborNodes(currentNode);
				for(unsigned int i = 0; i<neighborNodes->size(); i++)
				{
					if((*neighborNodes)[i]->status==CLOSED){continue;}
					IFNode* neighborNode = (*neighborNodes)[i];
					float tentG = currentNode->distance + (currentNode->point- neighborNode->point).length();
					if(neighborNode->status==UNDEFINED){
						insertOpen(neighborNode);
					}
					if(tentG < neighborNode->distance){
						neighborNode->next = currentNode;
						neighborNode->distance = tentG;
					}
				}
				sort(openSet.begin(),openSet.end(),[](const IFNode* x,const IFNode* y){if (x->distance<y->distance) {return true;}else {return false;}});
				if(currentNode==&(nodes[startID]))
				{
					break;
				}
			}
		}
		if(getNodeFromPoint(start)->status==CLOSED)
		{
			std::vector<Util::Point> path; reconstructPath(&nodes[startID],path);
			//calculate path length to get startDistance
			Util::Point prev=path[0];double tempD=0;
			for(unsigned int i=1;i<path.size();i++)
			{
				tempD+= (path[i]-prev).length();prev=path[i];
			}
			startDistance=tempD;
		}
		else {return false;}//if it's still not closed there's no path
		//and then expand the field beyond the margin
		IFNode * currentNode;
		while((openSet.size()>0))
		{//the open set is already sorted by distance
			currentNode = popOpen();
			insertClosed(currentNode);
			std::vector<IFNode*>* neighborNodes=getNeighborNodes(currentNode);
			for(unsigned int i = 0; i<neighborNodes->size(); i++)
			{
				if((*neighborNodes)[i]->status==CLOSED){continue;}
				IFNode* neighborNode = (*neighborNodes)[i];
				float tentG = currentNode->distance + (currentNode->point- neighborNode->point).length();
				if(neighborNode->status==UNDEFINED){
					insertOpen(neighborNode);
				}
				if(tentG < neighborNode->distance){
					neighborNode->next = currentNode;
					neighborNode->distance = tentG;
				}
			}
			sort(openSet.begin(),openSet.end(),[](const IFNode* x,const IFNode* y){if (x->distance<y->distance) {return true;}else {return false;}});
			if(currentNode->distance>startDistance+margin)
			{
				break;
			}
		}
	
		
	}
	bool computePath(Util::Point start,std::vector<Util::Point>& path,IFAgent* agent)
	{
		/*  //this is A* code to save time on initial computing of the path, but if we recalculate every time with the optimistic distance as heuristic, it would often need to recompute to give heuristics on squares previously ignored, and every time we'd need to sort the open set; either keep a value for the smallest distance(not f, just distance) in the open set, or just use Dijkstras to provide enough information in one go
		// if that point is not in closed set, do a backwards A* to that point, reuse open set but with new heuristic?
		//the goal node is put into the open set when the goal is created, no need to do it every time
		path.clear();
		this->start=start;
		int startID=gSpatialDatabase->getCellIndexFromLocation(start.x,start.z);
		int goalID=gSpatialDatabase->getCellIndexFromLocation(goal.x,goal.z);
		if(getNodeFromPoint(start)->status==CLOSED){reconstructPath(&nodes[startID],path);return true;}
		
		IFNode * currentNode;
		//sort open set again because a new start may be queried, with new heuristics
		sort(openSet.begin(),openSet.end(),[this](const IFNode* x,const IFNode* y){if (this->f(x)<this->f(y)) {return true;}else {return false;}});
		while((openSet.size()>0)){
		  currentNode = popOpen();
		  insertClosed(currentNode);
		  
		  std::vector<IFNode*>* neighborNodes=getNeighborNodes(currentNode);
		 
		  for(unsigned int i = 0; i<neighborNodes->size(); i++){
			if((*neighborNodes)[i]->status==CLOSED)
			{continue;}
			  IFNode* neighborNode = (*neighborNodes)[i];
			  float tentG = currentNode->distance + (currentNode->point- neighborNode->point).length();
			  //neighbor node is not in open set
			  if(neighborNode->status==UNDEFINED){
				insertOpen(neighborNode);
			 }
			  if(tentG < neighborNode->distance){
				neighborNode->next = currentNode;
				neighborNode->distance = tentG;
			  }
			}
			sort(openSet.begin(),openSet.end(),[this](const IFNode* x,const IFNode* y){if (this->f(x)<this->f(y)) {return true;}else {return false;}});
			if(currentNode==&(nodes[startID]))
			{
			  //std::cout << " path found, created " <<nodes.size() << " nodes, goal ID is "<<goalID<<", goal is"<<goal.x<<","<<goal.z<<std::endl;
				reconstructPath(currentNode,path);
				return true;  
			}
		}
		return false;
		*/
		//now use best first search to prepare a larger field for all future queries(and no need to sort for each different heuristic as different agents will keep calling it)
		path.clear();

		this->start=start;double startDistance=-1;
		int startID=gSpatialDatabase->getCellIndexFromLocation(start.x,start.z);
		int goalID=gSpatialDatabase->getCellIndexFromLocation(goal.x,goal.z);
		
		
		if(getNodeFromPoint(start)->status!=CLOSED)
		{
			IFNode * currentNode;
			while((openSet.size()>0))
			{//the open set is already sorted by distance
				currentNode = popOpen();
				insertClosed(currentNode);//should stop after the target's neighbors are updated! or later node paths may be invalid, as the current target can be a node on another path
				std::vector<IFNode*>* neighborNodes=getNeighborNodes(currentNode);
				for(unsigned int i = 0; i<neighborNodes->size(); i++)
				{
					if((*neighborNodes)[i]->status==CLOSED){continue;}
					IFNode* neighborNode = (*neighborNodes)[i];
					float tentG = currentNode->distance + (currentNode->point- neighborNode->point).length();
					if(neighborNode->status==UNDEFINED){
						insertOpen(neighborNode);
					}
					if(tentG < neighborNode->distance){
						neighborNode->next = currentNode;
						neighborNode->distance = tentG;
					}
				}
				sort(openSet.begin(),openSet.end(),[](const IFNode* x,const IFNode* y){if (x->distance<y->distance) {return true;}else {return false;}});
				if(currentNode==&(nodes[startID]))
				{
					break;
				}
			}
		}
		if(getNodeFromPoint(start)->status==CLOSED)
		{
			reconstructPath(&nodes[startID],path);
			//calculate path length to get startDistance
			Util::Point prev=path[0];double tempD=0;
			for(unsigned int i=1;i<path.size();i++)
			{
				tempD+= (path[i]-prev).length();prev=path[i];
			}
			startDistance=tempD;
			if((leader==NULL)||leaderDistance>=startDistance+5){leader=agent;leaderDistance=startDistance;leaderStart=start;}
			else if((startDistance>=leaderDistance)||(startDistance-tempD<5)&&( rightSideInXZPlane(goal-start)*(leaderStart-start)>0 ))
			{
				//don't switch leader
			}
			else{leader=agent;leaderDistance=startDistance;leaderStart=start;}
			return true;
		}
		else {return false;}//if it's still not closed there's no path
		
	}

std::vector<IFNode*>* getNeighborNodes(IFNode* currentNode){
	if(currentNode->neighbors!=NULL){ return currentNode->neighbors; }
	currentNode->neighbors=new std::vector<IFNode*>();
    int id = gSpatialDatabase->getCellIndexFromLocation(currentNode->point);
    unsigned int x; unsigned int z;
    gSpatialDatabase-> getGridCoordinatesFromIndex(id, x, z);
    unsigned  int maxX = gSpatialDatabase->getNumCellsX(); unsigned int maxZ = gSpatialDatabase->getNumCellsZ();
	int cellIndex;
	bool isClear[3][3];
	for (unsigned int i=x-1; i<=x+1; i++) {
		for (unsigned int j=z-1; j<=z+1; j++) {
			if((i>0)&&(i<maxX)&&(j>0)&&(j<maxZ))//seems there's an off by 1 error somewhere so I decide to stay off the edge
			{
				int Id = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				isClear[i-x+1][j-z+1]=nodes[Id].isClear(Id); //isClear implicitly creates and initializes it(but not its neighbors) if the node is not initialized, and caches the spatial database result
			}
			else{isClear[i-x+1][j-z+1]=false;}
		}
	}
	if(!isClear[1][1]){return currentNode->neighbors;}//if the grid is not traversable, it has no neighbors
	if(isClear[1][0]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x,z-1)]);}
	if(isClear[1][2]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x,z+1)]);}
	if(isClear[0][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x-1,z)]);}
	if(isClear[2][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x+1,z)]);}
	if(isClear[0][0]&&isClear[1][0]&&isClear[0][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x-1,z-1)]);}
	if(isClear[0][2]&&isClear[1][2]&&isClear[0][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x-1,z+1)]);}
	if(isClear[2][0]&&isClear[1][0]&&isClear[2][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x+1,z-1)]);}
	if(isClear[2][2]&&isClear[1][2]&&isClear[2][1]){currentNode->neighbors->push_back(&nodes[gSpatialDatabase->getCellIndexFromGridCoords(x+1,z+1)]);}
	//std::cout<<"neighbors: "<<currentNode->neighbors->size()<<std::endl;
	return currentNode->neighbors;
  }
  double heuristic(Util::Point current, double epsilon=1){
	  double h ;
	  //if(heuristic=="Euclidean")
		  h = sqrt( pow(current.x - start.x, 2) + pow(current.z - start.z, 2) );//backwards search with a cached start
	  //if(heuristic=="Manhattan")h = abs(current.x - goal.x) + abs(current.z - goal.z);
    return epsilon * h;
  }
  double f(const IFNode* n)
  {
	  return heuristic(n->point)+n->distance;
  }
  void insertClosed(IFNode* n)
  {
	  //when this is called n should have been open
	  n->status=CLOSED;
	  closedSet.push_back(n);
  }
  void insertOpen(IFNode* n)
  {
	  n->status=OPEN;
	  openSet.push_back(n);
	  //TODO: make_heap
  }
  void updateOpen(IFNode* n)
  {
	  //to do this n must save a reference to its position in Open and whenever the heap is updated corresponding references have to change - or just sort the whole vector for now
	  //we sort once for all neighbors of one open node, no need for this
  }
  IFNode* popOpen()
  {
	  IFNode* temp=openSet.front();
	  temp->status=UNDEFINED;
	  openSet.erase(openSet.begin());
	  return temp;
  }

  void reconstructPath(IFNode* currentNode, std::vector<Util::Point>& result){
    //std::cout<<"reconstruct path: start is" <<currentNode->point.x<<","<<currentNode->point.z<<std::endl;
	std::vector<IFNode*>* neighborNodes=getNeighborNodes(currentNode);
		//for(int i = 0; i<neighborNodes->size(); i++)
		//{
		//	std::cout<<"  neighbor: "<<(*neighborNodes)[i]->point.x<<","<<(*neighborNodes)[i]->point.z<<", g: "<<(*neighborNodes)[i]->distance<<std::endl;
		//}
    result.push_back(currentNode->point);//backward search parents point back to the goal, so push back forms the forward path
    IFNode * currParent = currentNode->next;
    while(currParent != NULL){
      result.push_back(currParent->point);
	  //debug
	  //std::vector<IFNode*>* neighborNodes=getNeighborNodes(currParent);
	  //std::cout<<"point: "<<currParent->point.x<<","<<currParent->point.z<<" neighbors:"<<neighborNodes->size()<<std::endl;
	  //std::vector<IFNode*> neighborNodes;
		
			
      currParent = currParent->next;
    }
  }

};


//the shared "flow field"
class IFField{
	std::vector<IFNode *> closedSet; 
	std::vector<IFNode *> openSet;
	std::unordered_map<int,IFNode> nodes;
	Util::Point start;
};


class IFAgent : public SteerLib::AgentInterface
{
public:
	IFAgent();
	~IFAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const {return _velocity; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return id_;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { return _goalQueue; }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for IFAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for IFAgent"); }
	void setParameters(SteerLib::Behaviour behave);


	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }

	
	
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
	float sf_preferred_speed=PERFERED_SPEED;
	bool useLeader;
	
	IFAgent* myLeader=NULL;
	
	std::vector<Util::Point> path;
	
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	static std::vector<IFGoal*> goalList;//goal.computePath(start,path);
	
	static IFField* field;
	
	bool computePath(Util::Point start,bool dontplan);
	IFGoal* myGoal=NULL;
	bool replan=false;
	int replan_wait=0;//to throttle the rate of replanning when there seems to be no path
	Util::Point lastProgressPoint;//to detect getting stuck for whatever reason - a few agents always get stuck no matter what
	int framesWithoutProgress=0;
	
	//weight is like agent's agressiveness, or social influence, it is not fixed but is interactive, a part of the value will be passed around to other agents, as agents that collide into others lose weight, and agents that waited or deviated from goal for too long gain weight; it will also be used in the interactive field itself if we have it
	//it should also affect the social force parameters? or not?
	
	double weight=1;float maxWeight;
	Util::Vector goalDirection;
	
	double tiredness_x=0;
	double tiredness_z=0;//to limit turbulence
	
protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	// void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);


	virtual SteerLib::EngineInterface * getSimulationEngine();


	/**
		 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
		 */
	void update(float timeStamp, float dt, unsigned int frameNumber);


	// Stuff specific to RVO
	// should be normalized
	// Util::Vector prefVelocity_; // This is the velocity the agent wants to be at
	// Util::Vector newVelocity_;
	size_t id_;
	IFAIModule * gModule;

//	SteerLib::EngineInterface * _gEngine;
	SteerLib::EngineInterface * gEngine;

	// Used to store Waypoints between goals
	// A waypoint is choosen every FURTHEST_LOCAL_TARGET_DISTANCE

private:
	// bool runLongTermPlanning();
	// bool reachedCurrentWaypoint();
	// void updateMidTermPath();
	// bool hasLineOfSightTo(Util::Point point);


	void calcNextStep(float dt);
	Util::Vector calcRepulsionForce(float dt);
	Util::Vector calcProximityForce(float dt);

	Util::Vector calcAgentRepulsionForce(float dt);
	Util::Vector calcWallRepulsionForce(float dt);

	Util::Vector calcWallNormal(SteerLib::ObstacleInterface* obs);
	std::pair<Util::Point, Util::Point> calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal);
	Util::Vector calcObsNormal(SteerLib::ObstacleInterface* obs);

	// For midterm planning stores the plan to the current goal
	// holds the location of the best local target along the midtermpath

	friend class IFAIModule;

};


#endif
