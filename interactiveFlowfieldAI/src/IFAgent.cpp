//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "IFAgent.h"
#include "IFAIModule.h"
// #include <math.h>

#include <vector>
#include <stack>
#include <set>
#include <map>
#include <unordered_map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
// #include "util/Geometry.h"

/// @file IFAgent.cpp
/// @brief Implements the IFAgent class.

#undef min
#undef max



using namespace Util;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

std::vector<IFGoal*>  IFAgent::goalList;
SteerLib::SpatialDataBaseInterface* IFGoal::gSpatialDatabase;
SteerLib::SpatialDataBaseInterface* IFNode::gSpatialDatabase;

IFField* IFAgent::field;

IFAgent::IFAgent()
{
	
	_enabled = false;
}

IFAgent::~IFAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * IFAgent::getSimulationEngine()
{
	return gEngine;
}

void IFAgent::setParameters(Behaviour behave)
{
	this->setParameters(behave);
}

void IFAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled==true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void IFAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;
	_waypoints.clear();
	_midTermPath.clear();

	sf_acceleration = gModule->sf_acceleration;
	sf_personal_space_threshold = gModule->sf_personal_space_threshold;
	sf_agent_repulsion_importance = gModule->sf_agent_repulsion_importance;
	sf_query_radius =gModule-> sf_query_radius;
	sf_body_force =gModule-> sf_body_force;
	sf_agent_body_force = gModule->sf_agent_body_force;
	sf_sliding_friction_force =gModule-> sf_sliding_friction_force;
	sf_agent_b = gModule->sf_agent_b;
	sf_agent_a = gModule->sf_agent_a;
	sf_wall_b = gModule->sf_wall_b;
	sf_wall_a =gModule-> sf_wall_a;
	sf_max_speed =gModule-> sf_max_speed;
	maxWeight =gModule-> maxWeight;
	useLeader=gModule->useLeader;
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);


	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;
			}
			else
			{
				_goalQueue.push(initialConditions.goals[i]);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; IFAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	computePath(_goalQueue.front().targetLocation, dont_plan);

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	 
	lastProgressPoint=position();//my way of detecting getting stuck
	//goalDirection is now saved for later
	if ( !_midTermPath.empty() )
	{
		//my updateLocalTarget
		Util::Point tmpTarget = this->_midTermPath.at(0);
		// std::cout << "Size of mid term path: " << this->_midTermPath.size() << std::endl;
		int i=0;
		for (i=0; (i < FURTHEST_LOCAL_TARGET_DISTANCE) &&
				i < this->_midTermPath.size(); i++ )
		{
			tmpTarget = this->_midTermPath.at(i);
			if ( this->hasLineOfSightTo(tmpTarget) )
			{
				this->_currentLocalTarget = tmpTarget;

			}
			else
			{ // test for RVO2D
				break;
			}
		}
		for (int j=0; (j < (i-1)) && (i > 2) && (this->_midTermPath.size() > 1 ); j++)
		{// remove points behind where the agent can see
			this->_midTermPath.pop_front();
		}
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						sf_preferred_speed
					)
				- velocity()
				)
				/
				sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif


	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// _gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void IFAgent::calcNextStep(float dt)
{

}

std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector IFAgent::calcProximityForce(float dt)
{
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + sf_query_radius),
				_position.x+(this->_radius + sf_query_radius),
				_position.z-(this->_radius + sf_query_radius),
				_position.z+(this->_radius + sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	IFAgent * tmp_agent;
	SteerLib::ObstacleInterface * tmp_ob;
	Util::Vector away = Util::Vector(0,0,0);
	Util::Vector away_obs = Util::Vector(0,0,0);

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<IFAgent *>(*neighbour);

			// direction away from other agent
			Util::Vector away_tmp = normalize(position() - tmp_agent->position());
			// std::cout << "away_agent_tmp vec" << away_tmp << std::endl;
			// Scale force
			// std::cout << "the exp of agent distance is " << exp((radius() + tmp_agent->radius()) -
				//	(position() - tmp_agent->position()).length()) << std::endl;
			double factor=1;if(tmp_agent->weight>weight){factor=MIN(tmp_agent->weight/weight,10);}else{factor=tmp_agent->weight/weight;}
		
			//a high-weight agent should repel others, but attract people that have a similar intended direction that are behind the leader
			//and an agent following a high weight leader should increase in weight to make it less likely for the line to be broken
			double cos1=goalDirection*tmp_agent->goalDirection;
			double cos2=tmp_agent->goalDirection*(-away);
			
			away+=away_tmp*(- MIN(cos1,cos2))*0.02*factor;
			if(MIN(cos1,cos2)>0){weight=MIN(weight+(MIN(cos1,cos2)*0.01*tmp_agent->weight),maxWeight);}
			//add a force to make an agent go slightly to the right when about to run into someone
			
			double d=(this->position()-tmp_agent->position()).length();Util::Vector v=(tmp_agent->position()-this->position());
			double x=((this->radius()*2+tmp_agent->radius()+sf_personal_space_threshold)-d)/d;//a bigger area of avoidance?
			double my=(goalDirection*v)/d;//if you are going against someone you should deflect to the right
			if((my>0)&&(x>0)){away+=rightSideInXZPlane(v)*my*x;}
			
			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			away = away +
					(
						away_tmp//removed factor;
						*
						(
							sf_agent_a
							*
							exp(
								(
									(
										(
											this->radius()
											+
											tmp_agent->radius()+sf_personal_space_threshold
										)
										-
										(
											this->position()
											-
											tmp_agent->position()
										).length()
									)
									/
									sf_agent_b
								)
							)


						)
						*
						dt
					);
			/*
			std::cout << "agent " << this->id() << " away this far " << away <<
					" distance " << exp(
							(
								(
									(
										radius()
										+
										tmp_agent->radius()
									)
									-
									(
										position()
										-
										tmp_agent->position()
									).length()
								)
								/
								sf_agent_b
							)
						) << std::endl;
						*/
		}
		else
		{
			// It is an obstacle
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			CircleObstacle * obs_cir = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if ( obs_cir != NULL && USE_CIRCLES)
			{
				// std::cout << "Found circle obstacle" << std::endl;
				Util::Vector away_tmp = normalize(position() - obs_cir->position());
				away = away +
						(
							away_tmp
							*
							(
									sf_wall_a
								*
								exp(
									(
										(
											(
												this->radius()
												+
												obs_cir->radius()
											)
											-
											(
												this->position()
												-
												obs_cir->position()
											).length()
										)
										/
										sf_wall_b
									)
								)


							)
							*
							dt
						);
			}
			else
			{
				Util::Vector wall_normal = calcWallNormal( tmp_ob );
				std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
					// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance

				Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
				// std::cout << "away_obs_tmp vec" << away_obs_tmp << std::endl;
				// away_obs = away_obs + ( away_obs_tmp * ( radius() / ((position() - min_stuff.second).length() * B ) ) );
				away_obs = away_obs +
						(
							away_obs_tmp
							*
							(
								sf_wall_a
								*
								exp(
									(
										(
											(this->radius()+sf_personal_space_threshold) -
											(
												this->position()
												-
												min_stuff.second
											).length()
										)
										/
										sf_wall_b
									)
								)
							)
							*
							dt
						);
			}
		}

	}
	return away + away_obs;
}

Util::Vector IFAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
			(sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + (sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}

Util::Vector IFAgent::calcAgentRepulsionForce(float dt)
{

	Util::Vector agent_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + sf_query_radius),
				_position.x+(this->_radius + sf_query_radius),
				_position.z-(this->_radius + sf_query_radius),
				_position.z+(this->_radius + sf_query_radius),
				(this));

	IFAgent * tmp_agent;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<IFAgent *>(*neighbour);
		}
		else
		{
			continue;
		}
		if ( ( id() != tmp_agent->id() ) &&
				(tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001)
			)
		{
			double factor=1;if(tmp_agent->weight>weight){factor=MIN(tmp_agent->weight/weight,10);}else{if(weight>=2*tmp_agent->weight)factor=0;else{factor=tmp_agent->weight/weight;} }
		agent_repulsion_force = agent_repulsion_force +
			( tmp_agent->computePenetration(this->position(), this->radius()) * sf_agent_body_force * dt) *
			normalize(position() - tmp_agent->position());//removed factor
			// normalized tangential force
		/*
			agent_repulsion_force = agent_repulsion_force +
					(
						(
							(-1*position()) - tmp_agent->position()
						)
						/
						(
							(-1*position()) - tmp_agent->position()
						).length()

					)*0.2;
					*/
			//TODO this can have some funny behaviour is velocity == 0
			Util::Vector tangent = cross(cross(tmp_agent->position() - position(), velocity()),
					tmp_agent->position() - position());
			tangent = tangent /  tangent.length();
			float  tanget_v_diff = dot(tmp_agent->velocity() - velocity(),  tangent);
			// std::cout << "Velocity diff is " << tanget_v_diff << " tangent is " << tangent <<
				//	" velocity is " << velocity() << std::endl;
			agent_repulsion_force = agent_repulsion_force +
			(sf_sliding_friction_force * dt *
				(
					tmp_agent->computePenetration(this->position(), this->radius())
				) * tangent * tanget_v_diff //removed factor

			);
		}

	}
	return agent_repulsion_force;
}

Util::Vector IFAgent::calcWallRepulsionForce(float dt)
{

	Util::Vector wall_repulsion_force = Util::Vector(0,0,0);


	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + sf_query_radius),
				_position.x+(this->_radius + sf_query_radius),
				_position.z-(this->_radius + sf_query_radius),
				_position.z+(this->_radius + sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
	{
		if ( !(*neighbour)->isAgent() )
		{
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if ( tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001 )
		{
			CircleObstacle * cir_obs = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if ( cir_obs != NULL && USE_CIRCLES )
			{
				// std::cout << "Intersected circle obstacle" << std::endl;
				Util::Vector wall_normal = position() - cir_obs->position();
				// wall distance
				float distance = wall_normal.length() - cir_obs->radius();

				wall_normal = normalize(wall_normal);
				wall_repulsion_force = wall_repulsion_force +
					((
						(
							(
									wall_normal
							)
							*
							(
								radius() +
								sf_personal_space_threshold -
								(
									distance
								)
							)
						)
						/
						distance
					)* sf_body_force * dt);

				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
					// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
						// std::endl;
				wall_repulsion_force = wall_repulsion_force +
				(
					dot(forward(),  rightSideInXZPlane(wall_normal))
					*
					rightSideInXZPlane(wall_normal)
					*
					cir_obs->computePenetration(this->position(), this->radius())
				)* sf_sliding_friction_force * dt;

			}
			else
			{
				Util::Vector wall_normal = calcWallNormal( tmp_ob );
				std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
					// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance
				wall_repulsion_force = wall_repulsion_force +
					((
						(
							(
									wall_normal
							)
							*
							(
								radius() +
								sf_personal_space_threshold -
								(
									min_stuff.first
								)
							)
						)
						/
						min_stuff.first
					)* sf_body_force * dt);
				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
					// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
						// std::endl;
				wall_repulsion_force = wall_repulsion_force +
				(
					dot(forward(),  rightSideInXZPlane(wall_normal))
					*
					rightSideInXZPlane(wall_normal)
					*
					tmp_ob->computePenetration(this->position(), this->radius())
				)* sf_sliding_friction_force * dt;
			}
		}

	}
	return wall_repulsion_force;
}

std::pair<Util::Point, Util::Point> IFAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */
Util::Vector IFAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector IFAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}

/*
void IFAgent::computeNeighbors()
{
	agentNeighbors_.clear();

	if (rvo_max_neighbors > 0) {
		// std::cout << "About to segfault" << std::endl;
		dynamic_cast<IFAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, rvo_neighbor_distance * rvo_neighbor_distance);
		// std::cout << "Made it past segfault" << std::endl;
	}
}*/


//using computePath instead of runLongtermPlanning.

bool IFAgent::computePath(Util::Point goal, bool dontplan)
{
	static int count=0;
	count++;
	if(count%100==0){std::cout << "path computed: "<<count <<"times" <<std::endl;}
	//clamp goal to avoid edge
	if(goal.x<=gSpatialDatabase->getOriginX())goal.x=gSpatialDatabase->getOriginX()+0.01f;
	if(goal.z<=gSpatialDatabase->getOriginZ())goal.z=gSpatialDatabase->getOriginZ()+0.01f;
	if(goal.x>=gSpatialDatabase->getOriginX()+gSpatialDatabase->getGridSizeX())goal.x=gSpatialDatabase->getOriginX()+gSpatialDatabase->getGridSizeX()-0.01f;
	if(goal.z>=gSpatialDatabase->getOriginZ()+gSpatialDatabase->getGridSizeZ())goal.z=gSpatialDatabase->getOriginZ()+gSpatialDatabase->getGridSizeZ()-0.01f;
	//if goal in goalList
	int id = gSpatialDatabase->getCellIndexFromLocation(goal);
	myGoal=NULL;
	//setup tolerance checking and check if your goal is actually visible from the approximate goal?
	//first let's check just how bad it is to sort the open set for each agent every time wee need to replan, versus doing a Dijkstra for each agent at the start?
	for(unsigned int i = 0; i<goalList.size(); i++)
	{
			if(goalList[i]->goalID==id){myGoal=goalList[i]; break;}
			
			else if((goalList[i]->goal-goal).length()<10.0f)//tolerance
			{
				//code modified from hasLineOfSight
				float dummyt;
				Util::Vector toAnotherGoal = goalList[i]->goal-goal;
				SpatialDatabaseItemPtr dummyObject;
				Ray lineOfSightTest;
				Util::Point tmp_pos=goal;
				lineOfSightTest.initWithLengthInterval(tmp_pos, toAnotherGoal);
				if(gSpatialDatabase->trace(lineOfSightTest,dummyt, dummyObject, NULL,true)==false) {myGoal=goalList[i];break;}//not blocked
			}
	}
	//else create it first
	
	if(myGoal==NULL)
	{
		myGoal=new IFGoal(id);goalList.push_back(myGoal);  
		std::cout << "creating new goal #"<<goalList.size() << "at" << goalList[goalList.size()-1]->goal << std::endl;
	}
	Util::Point pos=position();
	if(pos.x<=gSpatialDatabase->getOriginX())pos.x=gSpatialDatabase->getOriginX()+0.01f;
	if(pos.z<=gSpatialDatabase->getOriginZ())pos.z=gSpatialDatabase->getOriginZ()+0.01f;
	if(pos.x>=gSpatialDatabase->getOriginX()+gSpatialDatabase->getGridSizeX())pos.x=gSpatialDatabase->getOriginX()+gSpatialDatabase->getGridSizeX()-0.01f;
	if(pos.z>=gSpatialDatabase->getOriginZ()+gSpatialDatabase->getGridSizeZ())pos.z=gSpatialDatabase->getOriginZ()+gSpatialDatabase->getGridSizeZ()-0.01f;
	bool success=myGoal->computePath(pos,path,this);
	if(!success) return false;//should go to a closest place instead?
	_midTermPath.clear();_waypoints.clear();
	for(int i=1; i <  path.size(); i++)
	{
		_midTermPath.push_back(path.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(path.at(i));
		}
	}
	_waypoints.push_back(goal);
	return true;
}


void IFAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// std::cout << "rvo_max_speed " << rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &(dynamic_cast<IFAIModule *>(gModule)->gPhaseProfilers->aiProfiler) );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	
	// std::cout << "midtermpath empty: " << _midTermPath.empty() << std::endl;
	//Some agents seem trapped when their next local target was seen but is not visible and they could get stuck, so maybe replan when local target is not visible?
	
	if (!_midTermPath.empty() && framesWithoutProgress> 10)
	{
		framesWithoutProgress=0;computePath(_goalQueue.front().targetLocation, false);
	}
	if (!_midTermPath.empty() && !this->hasLineOfSightTo(_currentLocalTarget))
	{
		replan_wait++;
		if((replan_wait>10)||replan){replan_wait=0;replan=false;computePath(_goalQueue.front().targetLocation, false);}
	}

	bool goalVisible=this->hasLineOfSightTo(goalInfo.targetLocation);
	if ( ! _midTermPath.empty() && (!goalVisible) )
	{
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}
		// my own updateLocalTarget
		Util::Point tmpTarget = this->_midTermPath.at(0);
		// std::cout << "Size of mid term path: " << this->_midTermPath.size() << std::endl;
		int i=0;
		for (i=0; (i < FURTHEST_LOCAL_TARGET_DISTANCE) &&
				i < this->_midTermPath.size(); i++ )
		{
			tmpTarget = this->_midTermPath.at(i);
			if ( this->hasLineOfSightTo(tmpTarget) )
			{
				this->_currentLocalTarget = tmpTarget;

			}
			else
			{ // test for RVO2D
				break;
			}
		}
		// std::cout << "Can see to midTermPath point: " << i << std::endl;
		// This makes SF TOO GOOD
		for (int j=0; (j < (i-1)) && (i > 2) && (this->_midTermPath.size() > 1 ); j++)
		{// remove points behind where the agent can see
			this->_midTermPath.pop_front();
		}
		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{
		
		goalDirection = normalize(goalInfo.targetLocation - position());
		if((!goalVisible))
		{
			replan_wait++;
			if((replan_wait>20)||replan){replan_wait=0;replan=false;computePath(_goalQueue.front().targetLocation, false);}
		}
	}
	
	if(useLeader&&hasLineOfSightTo(myGoal->leader->position()))
	{ 
			goalDirection = normalize( (myGoal->leader->position() - position())/5+goalDirection);
			
	}
		
	// _prefVelocity = goalDirection * PERFERED_SPEED;
	Util::Vector prefForce = (((goalDirection * sf_preferred_speed) - velocity()) / (sf_acceleration/dt)); //assumption here
	prefForce = prefForce + velocity();
	// _velocity = prefForce;
	
	
	

	Util::Vector wallRepulsionForce = calcWallRepulsionForce(dt);
	Util::Vector agentRepulsionForce = sf_agent_repulsion_importance*calcAgentRepulsionForce(dt);
	
	
	Util::Vector repulsionForce=wallRepulsionForce+agentRepulsionForce;
	if ( repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		repulsionForce = velocity();
		// throw GenericException("SocialForces numerical issue");
	}
	Util::Vector proximityForce = calcProximityForce(dt);
// #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	// _velocity = _newVelocity;
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}
	
	
	
	//acceleration leads to tiredness
	Util::Vector oldv=_velocity;
	
	double nearGoalDiscount=1-1/(goalInfo.targetLocation - position()).length();
	_velocity = (prefForce) + repulsionForce + (goalVisible?nearGoalDiscount:1)*proximityForce;//ignore other forces when very close to the goal, because the weight avoidance can cause high-weight agents to get stuck around the goal in a vortex
	
	//to avoid getting pushed over the edge, and causing lots of collisions(seems agents over the edge can't do collision avoidance)
	//if(position().x<=gSpatialDatabase->getOriginX()+1)_velocity.x=MAX(_velocity.x,0);
	//if(position().z<=gSpatialDatabase->getOriginZ()+1)_velocity.z=MAX(_velocity.z,0);
	//if(position().x>=gSpatialDatabase->getOriginX()+gSpatialDatabase->getGridSizeX()-1)_velocity.x=MIN(_velocity.x,0);
	//if(position().z>=gSpatialDatabase->getOriginZ()+gSpatialDatabase->getGridSizeZ()-1)_velocity.z=MIN(_velocity.z,0);
	
	
	_velocity = clamp(velocity(), sf_max_speed);
	_velocity.x*=1/(1+tiredness_x);_velocity.z*=1/(1+tiredness_z);
	_velocity.y=0.0f;
	sf_sliding_friction_force=gModule->sf_sliding_friction_force*MAX(0.2,_velocity.length()/sf_preferred_speed);
	sf_wall_a=WALL_A*MAX(0.2,_velocity.length()/sf_preferred_speed);
	double accel_x=(_velocity-oldv).x;accel_x=MAX(accel_x,-accel_x);double accel_z=(_velocity-oldv).z;accel_z=MAX(accel_z,-accel_z);
	tiredness_x=MIN(accel_x*0.1+tiredness_x,50);tiredness_z=MIN(accel_z*0.1+tiredness_z,50);tiredness_x*=0.98;tiredness_z*=0.98;
	
	if((_velocity.length()<sf_preferred_speed*0.1)||tiredness_x+tiredness_z>10){framesWithoutProgress++;}
#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);

/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
		_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	 * Now do the conversion from IFAgent into the SteerSuite coordinates
	 */
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
					Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
							goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().lengthSquared() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	
	
	
	
		///////////////////////////////////////
	//my interactive weight - punish agents that block or bump into others
	//the one who triggers this code is assumed to be the offender, as collisions will be removed after being discovered if possible
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;int agentCount=0;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
			_position.x-(this->_radius + sf_query_radius),
			_position.x+(this->_radius + sf_query_radius),
			_position.z-(this->_radius + sf_query_radius),
			_position.z+(this->_radius + sf_query_radius),
			(this));
	SteerLib::AgentInterface * tmp_agent;
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	{
		if ( (*neighbour)->isAgent() ){agentCount++;tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);}else{continue;}
		if (id()!= tmp_agent->id())
		{
			IFAgent* a=dynamic_cast<IFAgent *>(*neighbour); double d=(this->position()-tmp_agent->position()).length();Util::Vector away=(this->position()-tmp_agent->position());
			double x=((this->radius()+tmp_agent->radius()+sf_personal_space_threshold)-d)/d;
			double my=(goalDirection*(-1*away))/d;//if you are going directly against another you should give up some weight, but agents who already have lots of weight are less likely to lose weight, to make sure a line that's already moving forward should not be easily cut off
			if(a && (x>0)&&(my>0)){double tempw=MIN(weight,1/weight);weight-=tempw*x*my;a->weight+=tempw*x*my;}//test, try to see what weight really does, before deciding how to update it
			
		}
	}
	weight*=MIN(1,1-(1-_velocity.length()/sf_preferred_speed)*0.5);//people already moving should get more weight, stuck ones should get less weight but slowly
	if(weight>maxWeight){weight=maxWeight;}
	if(weight<1){weight+=0.01;}
	
	//stuck elimination
	
}


void IFAgent::draw()
{

	if(_midTermPath.empty())
	{
		this->_color = Util::gRed;
	}
	else
	{
		if(myGoal->leader==this)this->_color = Util::gDarkCyan;
		else{this->_color = Util::gBlue;}
	}
	
	Util::DrawLib::drawLine(position(), position()+goalDirection*weight, Util::Color(0.5f, 0.5f, 0.5f), 2);
	Util::DrawLib::drawLine(position(), position()+Util::Vector(1,0,0)*tiredness_x, Util::Color(0.8f, 0.0f, 0.8f), 2);
	Util::DrawLib::drawLine(position(), position()+Util::Vector(0,0,1)*tiredness_z, Util::Color(0.8f, 0.0f, 0.8f), 2);
	
#ifdef ENABLE_GUI
	AgentInterface::draw();
	// if the agent is selected, do some annotations just for demonstration
if (gEngine->isAgentSelected(this)) {
		//replan=true;
		
	int currIndex = gSpatialDatabase->getCellIndexFromLocation(_position);
	unsigned int currGridCoordX; unsigned int currGridCoordZ;
	gSpatialDatabase-> getGridCoordinatesFromIndex(currIndex, currGridCoordX, currGridCoordZ);
	int gridMaxX = gSpatialDatabase->getNumCellsX(); int gridMaxZ = gSpatialDatabase->getNumCellsZ();
	for (unsigned int i=currGridCoordX-2; i<=currGridCoordX+2; i++) 
	{
		for (unsigned int j=currGridCoordZ-2; j<=currGridCoordZ+2; j++) {
			if(((j!=currGridCoordZ)||(i!=currGridCoordX))&&(i>=0)&&(i<=gridMaxX)&&(j>=0)&&(j<=gridMaxZ))
			{
				int Id = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				//get clear status
				if(myGoal->nodes[Id].isClear()==false){Util::DrawLib::drawCircle(myGoal->nodes[Id].point, Util::Color(0.5f, 0.0f, 0.5f),0.5f);continue;}
				if(myGoal->nodes[Id].next!=NULL){Util::DrawLib::drawLine(myGoal->nodes[Id].point, myGoal->nodes[Id].point+(myGoal->nodes[Id].next->point-myGoal->nodes[Id].point)/2, Util::Color(0.0f, 0.5f, 0.0f), 2);}
			}
		}
	}	
		

	//debug: show known walkable/unwalkable grids
	if(path.size()>0)
	{
		for(unsigned int i = 1; i<path.size(); ++i)
		{Util::DrawLib::drawLine(path[i-1], path[i], Util::Color(1.0f, 0.0f, 0.0f), 2);Util::DrawLib::drawCircle(path[i], Util::Color(0.5f, 0.5f, 0.0f),0.2f);}
		Util::DrawLib::drawCircle(path[path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}
	}
#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
					this->position()
					+
					(
						(
							dynamic_cast<AgentInterface*>(*neighbor)->position()
							-
							this->position()
						)
					/2), Util::Vector(1,0,0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while ( __oldPositions.size() > mostPoints )
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}

	for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}

