
Mail
COMPOSE
Labels
Inbox (3,111)
Starred
Important
Sent Mail
Drafts (41)
Circles
Friends
Family
Acquaintances
Following
Follow up
Misc
More

Move to Inbox More
2 of 10

Collapse all Print all In new window
A3
Inbox
x

Jason Ramirez <jasonra@scarletmail.rutgers.edu>
Oct 18

<<<<<<< HEAD
to QI
Hi Qi,
=======
class SocialForcesAgent : public SteerLib::AgentInterface
{
public:
//begin behaviours: need to store parameters for easy access
	bool err=false;//debug output
	bool should_update_behaviours=true;//update in the first updateAI call, and also when the goal is changed or something else wants the agent to update
	void updateBehaviours();
	std::string name;//need to save the name for pursue querying
	std::vector<SocialForcesAgent*> pursue_agents;std::vector<SocialForcesAgent*> evade_agents;//references to agents this agent should follow or evade; to avoid the trouble of checking the name each frame
	float pursue_force=1;float evade_force=1;float pursue_force_exp=-INFINITY;float evade_force_exp=0;bool pursue_force_normalize=false;bool evade_force_normalize=false;bool pursue_evade_force_normalize=true;//whether to normalize the two forces before of after combining them
	
	//wall following
	bool wall_following=false;bool clockwise=true;//meaning around the obstacle
	float wall_following_force=100;float wall_attracting_force=100;//should be large enough to override the goal seeking
	//TODO: support pursue/evade by id if no name is given, as in a group with popular and unpopular people
	
	////LEADER FOLLOWING VARS
	static std::vector<SocialForcesAgent*> leaders;
	SocialForcesAgent* leader;
	Util::Point leaderPosition;
	bool leader_following = false;
	bool follow_visible_leader=false;
	float avoiding_leaders_path_force=5;
	
//end behaviours
	SocialForcesAgent();
	~SocialForcesAgent();
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
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for SocialForcesAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for SocialForcesAgent"); }
	void setParameters(SteerLib::Behaviour behave);
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::SpatialDataBaseInterface spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}

	// bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);
	// void insertAgentNeighbor(const SteerLib::AgentInterface * agent, float &rangeSq) {throw Util::GenericException("clearGoals() not implemented yet for SocialForcesAgent");}
	// bool compareDist(SteerLib::AgentInterface * a1, SteerLib::AgentInterface * a2 );
>>>>>>> dc2c2d2ef0be691901c17c16b57f680cc3af1a11

Have you worked on A3? And if so, can you send me what you've done so far. I haven't really been able to work on it because I've been pretty sick the past few days. Also, if you went to lecture today, did they mention anything about the assignment deadlines?

QI DONG
Oct 18

to me
Sorry I didn't do it, my other project has begun... I remember the professor saying to someone that it should be Friday, but it hasn't changed yet so I'm no sure.


QI DONG
Oct 19

to Zachary, me
Sorry about not doing much for this assignment, I had to catch up on my job and other project since last Thursday. I just returned from class and had dinner, and will see what I can do. If you have any part done, or just partial code, please share them


QI DONG
Oct 20

to me, Zachary
I'm just starting out. I may write the maze solver first if you want to write something else. A small tip in case you haven't figured it out: the behaviors like pursue are defined in xml, and the format is like this:(insert the behavior in a child goal of the goal sequence of an agent or agentregion; the behaviour must have steeringaAlgorithm and Parameters)



<goalSequence>
<seekStaticTarget>
<targetLocation> <x>-90</x> <y>0</y> <z>30</z> </targetLocation>
<desiredSpeed>1.3</desiredSpeed>
<timeDuration>1000.0</timeDuration>
<Behaviour>
<SteeringAlgorithm>WallFollower</SteeringAlgorithm>
<Parameters>
<Parameter><key>test11</key>
<value>1</value></Parameter>
</Parameters>
</Behaviour>
</seekStaticTarget>
</goalSequence>


QI DONG
Oct 20

to me, Zachary
And we can get the behaviours with Behaviour::getSteeringAlg() (just a string) and  std::vector<BehaviourParameter> Behaviour::getParameters(), and a BehaviourParameter has public fields key and value, both are strings,


QI DONG
Oct 20

to me, Zachary
and to get the behaviour in UpdateAI where we need it, use the current goal; like this

SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
//todo:debug - there are many agents and most don't have a behavior, so only print those that have
if(test && (goalInfo.targetBehaviour.getSteeringAlg()!=""))
{
    test=false;
    std::cerr << goalInfo.targetBehaviour.getSteeringAlg() << ", " <<std::endl;
    int q;
    for (q=0; q < goalInfo.targetBehaviour.getParameters().size(); q++ )
    {
        std::cerr << " Parameter" << std::endl;
        std::cerr << "\tkey: " << goalInfo.targetBehaviour.getParameters().at(q).key
        << " value " << goalInfo.targetBehaviour.getParameters().at(q).value << std::endl;
    }
    }
    
    
    QI DONG
    AttachmentsOct 20
    
    to me, Zachary
    
    Attachments area
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 20
    
    to QI, Zachary
    The changes made are only in the calcWallRepulsionForce method, right?
    
    
    QI DONG
    Oct 20
    
    to me, Zachary
    I'm modifying that and it should be the only modification that matters. But I might have changed stuff to debug. You can remove other modifications.
    
    
    QI DONG
    AttachmentsOct 20
    
    to me, Zachary
    I think my wall follower is OK. Sometimes an agent gets squeezed into a wall if the wall-following force is large, but too small a force will cause it to freeze so I left it like that. The balance between attraction to a wall and following along a wall is tricky, using the default of 100, 100 is fine for that test case. What else do you want me work on?
    
Note: my new behavior code is put between comments like //begin behaviour wallfollower and //end behavior, to be more readable.
    
    2 Attachments
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 20
    
    to QI, Zachary
    I'm working on leader following (group behavior),  so I guess you can work on another one of the individual behaviors.
    
    
    QI DONG
    Oct 20
    
    to me, Zachary
    OK, but do you know what exactly do the behaviors mean besides wall follower(it sounds clear so I did it first)? What is expected in pursue and evade - does the behaviour do both at the same time? And how to specify an agent as a moving target? And I don't know what is growing spiral or Unaligned Collision Avoidance.
    
    By the way, Zachary, you don't have to do the C++ homework(though it's a good idea to at least read and run our code) if you would rather work on Unity. We need more hands in Unity, especially for B3. Behavior Trees are tricky! Please start looking into it and at least get going and solve the basic problems before we move on to that.
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 20
    
    to QI, Zachary
    Some agents pursue and others evade at the same time I'm pretty sure. Can't you just set some parameter in the xml that indicates whether it is a target, and give its parameters to a pursue agent? I think growing spiral is when an agent or agents start in the center and then spiral outward.
    
http://www.red3d.com/cwr/steer/ has more info on pursue and evade, and on unaligned collision avoidance.
    
    
    Zachary Iuso
    Oct 20
    
    to me, QI
    I just got out of an exam. Now that it's over and I don't have to preoccupy myself with studying, I can start on unity.
    
    
    QI DONG
    Oct 21 (13 days ago)
    
    to Zachary, me
    I wonder how you can let another agent know the identity or role of another agent? I saw no good way of doing that if we can only change the social agent files, as AgentInterface doesn't support any given name of the agent. There's an ID, sure, and you might take advantage of that as a module automatically gives ascending ids, so you can make everyone follow the #1 leader, but in pursue there's no straightforward solution.
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 21 (13 days ago)
    
    to QI, Zachary
    What if you make all the agents with an odd id, x, pursue an agent with an even id, x + 1?
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 21 (13 days ago)
    
    to QI
    Do you want to meet up today? I need some help understanding the code, I know pretty much everything I need to do for the behavior to work, but I'm not entirely sure how to put it into code.
    
    
    QI DONG
    Oct 21 (13 days ago)
    
    to me
    Sure, you can come anytime. Call me at 848 239 8646 when you arrive in case I'm not reading email.
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 21 (13 days ago)
    
    to QI
    Ok, I'll  be there at about 5.
    
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    Hi here's my new code for pursue and evade. Note that a lot has changed - see comments like //begin behavior(behaviour)
    
    3 Attachments
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    AttachmentsOct 21 (13 days ago)
    
    to QI
    
    Attachments area
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    AttachmentsOct 21 (13 days ago)
    
    to QI
    
    Attachments area
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    AttachmentsOct 21 (13 days ago)
    
    to QI
    
    Attachments area
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    My new leader following
    
    3 Attachments
    
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    
    Attachments area
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    
    Attachments area
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    AttachmentsOct 21 (13 days ago)
    
    to QI, Zachary
    pursue and evade, and wallfollower videos
    wallfollowing_a3.mov
    
    2 Attachments
    
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    
    Attachments area
    
    QI DONG
    AttachmentsOct 21 (13 days ago)
    
    to me, Zachary
    
    Attachments area
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 21 (13 days ago)
    
    to QI
    total number of agents: 16
    
    avg. number of collisions per agent: 0
    
    average time spent by one agent: 84.3295
    
    average energy spent by one agent: 0
    
    sum of instantaneous accelerations: 0
    
    (alpha, beta, gamma, delta) weights: (50,1,1,1)
    
    weighted sum: 50*0 + 1*84.3295 + 1*0 + 1*0 = 84.3295
    
    final score: 84.3295
    
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 22 (12 days ago)
    
    to QI
    ​
    crowdfollowing2_a3.mov
    ​
    
    Attachments area
    
    QI DONG
    Oct 22 (12 days ago)
    
    to me
    Hi are you home yet?
    Please send the package for the B2 assignment you worked on to all of us, so we can use some code later. And remember to follow up on the other group member. I expect to be busy next week so may not help you too much on B3.
    
    
    Jason Ramirez <jasonra@scarletmail.rutgers.edu>
    Oct 22 (12 days ago)
    
    to Zachary, QI
    ​
    b2.unitypackage
    
    Attachments area
    
    Click here to Reply, Reply to all, or Forward
    Using 2.56 GB
    Program Policies
    Powered by Google
    Last account activity: 1 minute ago
    Details
    People (2)
    QI DONG
    Add to circles
    
    Show details
    
    
    
    //
    // Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
    // See license.txt for complete license.
    //
    
    
#ifndef __SocialForces_AGENT__
#define __SocialForces_AGENT__
    
    /// @file SocialForcesAgent.h
    /// @brief Declares the SimpleAgent class.
    
#include <queue>
#include <list>
#include "SteerLib.h"
    // #include "SimpleAgent.h"
    // #include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
    
    
    /**
     * @brief Social Forces Agent stuff
     *
     *
     */
    
    // #define DRAW_ANNOTATIONS 1
    // #define DRAW_HISTORIES 1
    // #define DRAW_COLLISIONS 1
    
    
    class SocialForcesAgent : public SteerLib::AgentInterface
    {
    public:
        //begin behaviours: need to store parameters for easy access
        bool err=false;//debug output
        bool should_update_behaviours=true;//update in the first updateAI call, and also when the goal is changed or something else wants the agent to update
        void updateBehaviours();
        std::string name;//need to save the name for pursue querying
        std::vector<SocialForcesAgent*> pursue_agents;std::vector<SocialForcesAgent*> evade_agents;//references to agents this agent should follow or evade; to avoid the trouble of checking the name each frame
        float pursue_force=1;float evade_force=1;float pursue_force_exp=-INFINITY;float evade_force_exp=0;bool pursue_force_normalize=false;bool evade_force_normalize=false;bool pursue_evade_force_normalize=true;//whether to normalize the two forces before of after combining them
        
        //wall following
        bool wall_following=false;bool clockwise=true;//meaning around the obstacle
        float wall_following_force=100;float wall_attracting_force=100;//should be large enough to override the goal seeking
        //TODO: support pursue/evade by id if no name is given, as in a group with popular and unpopular people
        
        ////LEADER FOLLOWING VARS
        static std::vector<SocialForcesAgent*> leaders;
        SocialForcesAgent* leader;
        Util::Point leaderPosition;
        bool leader_following = false;
        bool follow_visible_leader=false;
        float avoiding_leaders_path_force=5;
        
        //end behaviours
        SocialForcesAgent();
        ~SocialForcesAgent();
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
        void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for SocialForcesAgent"); }
        void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for SocialForcesAgent"); }
        void setParameters(SteerLib::Behaviour behave);
        /// @name The SteerLib::SpatialDatabaseItemInterface
        /// @brief These functions are required so that the agent can be used by the SteerLib::SpatialDataBaseInterface spatial database;
        /// The Util namespace helper functions do the job nicely for basic circular agents.
        //@{
        bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
        bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
        float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
        //@}
        
        // bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);
        // void insertAgentNeighbor(const SteerLib::AgentInterface * agent, float &rangeSq) {throw Util::GenericException("clearGoals() not implemented yet for SocialForcesAgent");}
        // bool compareDist(SteerLib::AgentInterface * a1, SteerLib::AgentInterface * a2 );
        
    protected:
        /// Updates position, velocity, and orientation of the agent, given the force and dt time step.
        // void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);
        
        SocialForcesParameters _SocialForcesParams;
        
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
        SteerLib::ModuleInterface * rvoModule;
        
        SteerLib::EngineInterface * _gEngine;
        
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
        
        friend class SocialForcesAIModule;
        
    };
    
    
#endif
