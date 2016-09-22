//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//TODO
	//debug: print all points data
	/* static bool printed=false;
	if(!printed)
	{
		std::cerr <<"Start point pos: " << controlPoints.begin()->position; std::cerr <<"End point pos: " << controlPoints[controlPoints.size()-1].position << "Size: " << controlPoints.size();
		for(unsigned int i =0; i < controlPoints.size(); i++) 
		{
			std::cerr <<"Point #" << i << ", pos: " << controlPoints[i].position << "tangent: " << controlPoints[i].tangent;
		}
		printed=true;
	} */
	
	
	//debug: draw the axes
	//DrawLib::drawLine(Point(0,0,0), Point(10,0,0), Util::Color(0.9,0,0),curveThickness);
	//DrawLib::drawLine(Point(0,0,0), Point(0,10,0), Util::Color(0,0.9,0),curveThickness);
	//DrawLib::drawLine(Point(0,0,0), Point(0,0,10), Util::Color(0,0,0.9),curveThickness);
	
	if (!checkRobust())return;
	static bool OK=true;
	Point currentPoint,nextPoint;//in a step
	for(unsigned int i =1; i < controlPoints.size(); i++) 
	{
		if(i>1){DrawLib::drawLine(currentPoint,controlPoints[i-1].position, curveColor,curveThickness);}//to ensure the curve always passes over control points
		currentPoint=controlPoints[i-1].position;
		
		//debug: drawing all points and tangents to see why my curves are wrong
		//DrawLib::drawCircle(currentPoint, curveColor, 1, 20);
		//DrawLib::drawLine(currentPoint, currentPoint+controlPoints[i-1].tangent*10, curveColor,curveThickness);
		
		for(float t=controlPoints[i-1].time+window;t<controlPoints[i].time;t+=window)
		{
			if(!calculatePoint(nextPoint,t)){if(OK){OK=false;std::cerr <<"Error: point not found at time: " << t ;}return;}
			DrawLib::drawLine(currentPoint, nextPoint, curveColor,curveThickness);
			currentPoint=nextPoint;
		}
		
	}
	//final line to last point
	DrawLib::drawLine(currentPoint,controlPoints[controlPoints.size()-1].position, curveColor,curveThickness);
	//debug
	//unsigned int i=controlPoints.size()-1;
	//DrawLib::drawCircle(controlPoints[i].position, curveColor, 1, 20);
	//DrawLib::drawLine(controlPoints[i].position, controlPoints[i].position+controlPoints[i].tangent*10, curveColor,curveThickness);
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
	
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
//TODO
	sort(controlPoints.begin(),controlPoints.end(),[](const  CurvePoint& x,const CurvePoint& y){if (x.time<y.time) {return true;}else {return false;}});
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	//TODO
	if(controlPoints.size()<2)
	{
		std::cerr <<"Error: less than two points given: " << controlPoints.size();
		return false;
	}
	float prevTime=controlPoints[0].time;
	for(unsigned int i =1; i < controlPoints.size(); i++) 
	{
		if(controlPoints[i].time<prevTime)
		{
			std::cerr <<"Error: points earlier than previous one: "<<controlPoints[i].time<<" previous:"<< prevTime;
			return false;
		}
		if(controlPoints[i].time==prevTime)
		{
			std::cerr <<"Error: points at same time as previous one, I'm changing the time a little bit to accomodate: "<<controlPoints[i].time<<" previous:"<< prevTime;
			for (unsigned int j=i;j<controlPoints.size();j++){controlPoints[j].time+=1;}
		}
		prevTime=controlPoints[i].time;
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	//TODO
	for(unsigned int i =1; i < controlPoints.size(); i++) 
	{
		if(controlPoints[i].time>time)
		{
			nextPoint=i;return true;
		}
	}
	std::cerr <<"Error: control point not found for time: "<< time;
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	//TODO
	float t1,t2,t;Point p1,p2,s1,s2,a,b,c,d;Vector v1,v2;//a,b,c,d are vectors for coefficients for each of x,y,z
	unsigned int i=nextPoint;
	t1=controlPoints[i-1].time;t2=controlPoints[i].time;
	p1=controlPoints[i-1].position;p2=controlPoints[i].position;
	v1=controlPoints[i-1].tangent;v2=controlPoints[i].tangent;s1=Point(v1.x,v1.y,v1.z)*(t2-t1);s2=Point(v2.x,v2.y,v2.z)*(t2-t1);
	//velocities should change because time interval is normalized
	t=(time-t1)/(t2-t1);
	newPosition=(((2*t-3)*t*t)+1)*p1+(-2*t+3)*t*t*p2+((t-2)*t+1)*t*s1+(t-1)*t*t*s2;//see page 64 of curve slides, time normalized to between 0-1

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//TODO: tension set to lower because my CM curve looks different from the examples and more like hermite curves
	float tau=0.05;
	float t1,t2,t;Point p1,p2,s1,s2,a,b,c,d;Vector v1,v2;//a,b,c,d are vectors for coefficients for each of x,y,z
	unsigned int i=nextPoint;
	t1=controlPoints[i-1].time;t2=controlPoints[i].time;
	p1=controlPoints[i-1].position;p2=controlPoints[i].position;
	v1=controlPoints[i-1].tangent;v2=controlPoints[i].tangent;
	if(i==1){v1=controlPoints[i-1].tangent;}else{v1=controlPoints[i].position-controlPoints[i-2].position;}
	if(i==controlPoints.size()-1){v2=controlPoints[i].tangent;}else{v2=controlPoints[i+1].position-controlPoints[i-1].position;}
	
	s1=Point(v1.x,v1.y,v1.z)*(t2-t1)*tau;s2=Point(v2.x,v2.y,v2.z)*(t2-t1)*tau;
	t=(time-t1)/(t2-t1);
	newPosition=(((2*t-3)*t*t)+1)*p1+(-2*t+3)*t*t*p2+((t-2)*t+1)*t*s1+(t-1)*t*t*s2;//see page 64 of curve slides, time normalized to between 0-1
	
	// Return result
	return newPosition;
}