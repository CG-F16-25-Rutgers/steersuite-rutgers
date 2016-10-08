#include "obstacles/GJK_EPA.h"

static double delta[16][4];
static double deltaSum[16];
static bool deltaComputed[16][4];
//only one instance doing this at any moment, right?
//deltasum is the sum of all delta(i) for a subset; when they are computed, individual deltas are computed as needed
//instead of allocating and deallocating, I use a static array for dimension=3
//there are at most 4 points in the simplex, so n=4, we need an array d[16][4]

double computeDelta(const std::vector<bool>& subset,unsigned int index,const std::vector<Util::Vector>& simplex)
{
	//for one subset; recurse to smaller subsets and save the result if necessary
	//subset index is a bitmask to avoid more complicated storage
	unsigned int bitmask=0, cardinal=0; int min=-1;
	for (unsigned int i=0;i<subset.size();i++)
	{
		if(subset[i])
		{
			bitmask+=1<<i;cardinal++;if((min==-1)&&(i!=index)){min=i;}
		}//find the first element of the smaller subset to be used in the formula
	}
	
	
	
	if(deltaComputed[bitmask][index]){return delta[bitmask][index];}
	//otherwise compute delta
	if(cardinal==1)
	{
		if(subset[index])
		{
			delta[bitmask][index]=1;deltaComputed[bitmask][index]=true;return 1;
		}
		else
		{
			#ifdef DEBUG 
			//std::cerr <<"Queried index doesn't exist in subset: " << index <<std::endl;
			#endif
		}
	}
	std::vector<bool> subset2(subset);
	subset2[index]=false;
	//this is the smaller subset to recurse on
	double sum=0;
	for(unsigned int j=0;j<subset2.size();j++)
	{
		if(subset2[j])
		{
			sum+=computeDelta(subset2,j,simplex)*(simplex[j]*simplex[min]-simplex[j]*simplex[index]);
		}
	}
	delta[bitmask][index]=sum;deltaComputed[bitmask][index]=true;
	/*
	#ifdef DEBUG
	std::cerr << "Delta " << bitmask << ", " << index <<  "  is " << delta[bitmask][index] << std::endl;
	#endif
	*/
	return sum;
}

void getSubset(unsigned int bitmask,unsigned int length,std::vector<bool>& subset)
{
	for(unsigned int i=0;i<length;i++)
	{
		if(bitmask&(1<<i)){subset[i]=true;}else{subset[i]=false;}
	}
}
 int instances=0;
Util::Vector findClosest(std::vector<Util::Vector>& simplex)
{ 
	//subalgorithm at page 197: suppose there are n points in the simplex. Allocate 2^n slots, with n+1 real numbers each. Consider an n-bit bitmask as meaning a subset of these n vertices. Compute Delta(i,subset) for each non-empty subset and each point i by the recursive formula: if the subset contains only the one element i, then delta(i,subset)=1; if the subset contains i and some other elements(denoted by subset2) then delta(i,subset)=sum over all j in subset2 of delta(j, subset2)*(point_j dot point_k - point_j dot point_i) where dot is the dot product of the vectors of the points' position, and k is the point in subset2 with the smallest index(actually choosing any point in subset2 gets the same result). If subset doesn't contain i, I don't think delta(i, subset) is defined, or maybe it defaults to 0. Delta (subset) is the sum of delta(i, subset) of all i in subset. The convex combination of a subset of simplex vertices that gives the closest point to origin, is given by the first subset that has the properties: delta(subset)>0(meaning the points are affinely independent), delta(i, subset)>0 for all i in it(the combination is convex), and delta(subset2,j)<=0 for any bigger subset2 that only has one more element j(the addition of another point doesn't contribute to getting closer to the origin). and the coefficients of the convex combination, lambda(i) is given by delta(i, subset)/delta(subset).
	#ifdef DEBUG
	instances++;
	//std::cerr << "entering delta calculations; simplex size is" << simplex.size()  << std::endl;
	for(unsigned int index=0;index<simplex.size();index++)
	{
		//std::cerr << "Simplex point " << index <<  "  is " << simplex[index].x <<"," <<simplex[index].y<< "," <<simplex[index].z << std::endl;
	}
	#endif
	for(int i=0;i<16;i++)
	{
		for(int j=0;j<4;j++)
		{
			delta[i][j]=0;deltaComputed[i][j]=false;
		}
	}
	unsigned int count=simplex.size();unsigned int subsets=1<<count;
	std::vector<bool> subset(count);
	for (unsigned int bitmask=1;bitmask<subsets;bitmask++)
	{
		getSubset(bitmask,count,subset);
		deltaSum[bitmask]=0;
		for(unsigned int index=0;index<count;index++)
		{
			//compute the ith subset's jth delta, recursing as necessary - i=0 is ignored because the empty subset is unused
			//once a delta is computed, it is marked as computed and saved for future use
			if(subset[index]){deltaSum[bitmask]+=computeDelta(subset,index,simplex);}
			//simplex needed for the points dot product
			
		}
		#ifdef DEBUG
		//std::cerr << "Delta " << bitmask <<  "  is " << deltaSum[bitmask] << std::endl;
		#endif
	}

	//after all is done, find the subset you want: delta(subset)>0(meaning the points are affinely independent), delta(i, subset)>0 for all i in it(the combination is convex), and delta(subset2,j)<=0 for any bigger subset2 that only has one more element j(the addition of another point doesn't contribute to getting closer to the origin). and the coefficients of the convex combination, lambda(i) is given by delta(i, subset)/delta(subset).
	bool OK;unsigned int bitmask;
	for (bitmask=1;bitmask<subsets;bitmask++)
	{
		getSubset(bitmask,count,subset);
		if(deltaSum[bitmask]<=0){continue;}
		OK=true;
		for(unsigned int index=0;index<count;index++)
		{
			if((subset[index]==true)&&(delta[bitmask][index]<=0)){OK=false;break;}
		}
		if(!OK){continue;}
		for(unsigned int index=0;index<count;index++)
		{
			if(subset[index]==false){if(delta[bitmask+(1<<index)][index]>0){OK=false;break;}}//test all bigger subsets that has one more element
		}
		if(!OK){continue;}
		if(OK)
		{
			//std::cerr << "Simplex "<<bitmask<<" accepted!"<< std::endl;
			break;
		} 
	}
	if(OK)
	{
		//found combination;
		//return the closest point, and also prepare the simplex for next step by removing unused points that didn't contribute in the combination. the simplex is changed.
		std::vector<Util::Vector> newSimplex;
		Util::Vector v;
		for(unsigned int index=0;index<count;index++)
		{
			if(subset[index]){newSimplex.push_back(simplex[index]);v+=simplex[index]*delta[bitmask][index]/deltaSum[bitmask];}
			//else{delete &(simplex[index]);}
		}
		//delete &simplex;//TODO: is this correct?
		simplex=newSimplex;
		//delete &subset;
		#ifdef DEBUG
		//std::cerr <<"New Simplex size " << newSimplex.size() <<std::endl;
		#endif
		return v;
	}
	else
	{
		#ifdef DEBUG
		//std::cerr <<"Closest point in simplex to origin is not found!" <<std::endl;
		instances--;
		static int failure=0;
		failure++;
		if(failure>10){exit(0);}
		#endif
	}
	
}
double getSupport(const Util::Vector& direction, Util::Vector& support,const std::vector<Util::Vector>& A)
{
	double max=-INFINITY;
	for(unsigned int i =0; i < A.size(); i++)
	{
		if(A[i]*direction>max){max=A[i]*direction;support=A[i];}
	}
	return max;
}
double getMDSupport(const Util::Vector& direction, Util::Vector& support,const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B)
{
	//TODO:pseudocode
	Util::Vector support1; Util::Vector support2;double max;Util::Vector origin;
	max=getSupport(direction,support1,A)-getSupport(origin-direction,support2,B);//??
	support=support1-support2;
	return max;
}
bool GJK(std::vector<Util::Vector>& simplex,const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B)
{
	//1. set starting simplex to a random point(or a random set of points)
	//simplex starts out empty, and is changed, but the object is the same vector
	Util::Vector* support=new Util::Vector();//we reuse this variable to hold many support objects so this is a pointer
	Util::Vector origin;
	getMDSupport(A[0], *support,A,B);//TODO: just use the first point for debugging
	simplex.push_back(*support);
	int iterations=0;
	while(true)
	{
		iterations++;
		if(iterations>7)
		{
			//std::cerr <<"Too many iterations in GJK!!" <<std::endl;
			return false;
		}
		//2. find a point v in simplex closest to origin: TODO: check each open subset given by a subset of the vertices in the simplex(at most 15 for 4 points), and see if they contain the support by a complicated subalgorithm (see below)
		Util::Vector v=findClosest(simplex);
		
		//3. see if (support of -v in MD) dot v is less than |v|^2; if not, v is the closest point.	TODO: support of -v in MD, which is just the support of -v in A minus support of v in B, which can be found by iterating over all vertices in the shape and computing the dot product.
		//if v is the origin, it means they intersect and we should run EPA; if not, they don't intersect.
		//TODO
		#ifdef DEBUG
		//std::cerr <<"Square of the length of the closest point to origin in MD: " << v*v <<std::endl;
		#endif
		if(v*v<_UTIL_GEOMETRY_EPSILON)
		{
			return true;//do EPA
		}
		support=new Util::Vector();//the old support has become a part of the simplex so we don't delete it
		getMDSupport(origin-v,*support,A,B);
		#ifdef DEBUG
		//std::cerr <<"V is: " << v.x << ","<< v.y << ","<< v.z <<std::endl;
		//std::cerr <<"New support point in that direction: " << support->x << ","<< support->y << ","<< support->z <<std::endl;
		#endif
		if((*support)*v>0)//TODO: I changed this, meaning they are on the same side of the origin,  not sure why suport*v<v*v is the original formula
		{
			return false;//no collision
		}

		//4. if yes, the support should be added to the simplex to give a closer point to the origin in the simplex, and the old simplex should have points that don't contribute the the closest point removed to make sure the new simplex still has at most m+1 points; TODO: how to remove the unnecesaary points? the useful points are given by the convex combination of points in the simplex found in step 2, where only some points contribute to the result, and unused points can be discarded.
		//TODO
		simplex.push_back(*support);
		
		
		//5. go to 2
	}
	
}



//EPA part

struct closestEdge{
    Util::Vector normal;//normal vector from edge to origin
    double distance;// distance from origin
    int index; //index in simplex
};

//get vector normal to a and b: ab x(a x ab) = -ab(a dot ab) + a(ab dot ab)
//Util::Vector tripleProduct(Util::Vector& a, Util::Vector& b, Util::Vector& normal)
//{ 
//	return -1*(b-a)*(a*(b-a)) + a*((b-a) * (b-a));
//}

//To get the closest distance from the origin to a line segment in 3D, I can use a simplification of the subalgorithm in the paper:
//simplex is A,B, delta(A,A)=1, d(B,B)=1,d(A,AB)=B*(B-A);d(B,AB)=A*(A-B);
//d=B*(B-A)+A*(A-B); if A*(A-B)<=0, B doesn't contribute so the point is A; if B*(B-A)<0, the point is B; otherwise you want 
//(A*(B*(B-A)) + B* (A*(A-B)))/d
Util::Vector closestPointOnSegment(const Util::Vector& a, const Util::Vector& b)
{
	if(a*(a-b)<0){return Util::Vector(b);}
	if(b*(b-a)<0){return Util::Vector(a);}
	return (a*(b*(b-a)) + b* (a*(a-b)))/((b*(b-a))+(a*(a-b)));
}

//gets the edge closest to the origin
void findClosestEdge(std::vector<Util::Vector>& simplex, closestEdge& edge)
{
    edge.distance = DBL_MAX;
	unsigned int i,j,ai,bi;
    for(i=0; i<simplex.size(); i++)
    {
        //next point on the simplex
        if(i==(simplex.size()-1)){
           j = 0;
        }
        else{
            j = i+1;
        }
        //get pair of consecutive simplex points
        Util::Vector a = simplex.at(i); Util::Vector b = simplex.at(j);
        //Util::Vector normal=tripleProduct(a,b);
        //normalize
        //normal = normal/sqrt(normal*normal);
        
        Util::Vector closest = closestPointOnSegment(a,b);
		double distance = sqrt(closest*closest);
        if(distance < edge.distance){
            edge.distance = distance;
            edge.index = j;
			ai=i;bi=j;
            edge.normal = closest;//if closest point is not the normal(is one of the endpoints), it should be OK because that edge can't be the closest anyway, it won't be subdivided
        }
    }
	//std::cerr <<ai<<", "<<bi<<" edge is: " << simplex[ai].x <<", "<<simplex[ai].z<<"; "<<simplex[bi].x<<","<< simplex[bi].z<<", Normal:"<<edge.normal.x<<","<<edge.normal.z<<std::endl;
    
}


void EPA(const std::vector<Util::Vector>& A,const std::vector<Util::Vector>& B,std::vector<Util::Vector>& simplex,float& return_penetration_depth,Util::Vector& return_penetration_vector)
{
	Util::Vector* support;
	int iterations=0;
	double old_distance=0;
    while (1) {
		#ifdef DEBUG
		//std::cerr << "EPA: simplex size is" << simplex.size()  << std::endl;
		for(unsigned int index=0;index<simplex.size();index++)
		{
			//std::cerr << "Simplex point " << index <<  "  is " << simplex[index].x <<"," <<simplex[index].y<< "," <<simplex[index].z << std::endl;
		}
		#endif
        //get edge closest to the origin on the Minkowski Difference (A-B), using the current simplex, using struct: closestEdge
		iterations++;
        closestEdge edge;
        findClosestEdge(simplex, edge);
		
		
        // get new support point in the direction of edge normal
        support=new Util::Vector;
        double max = getMDSupport(edge.normal, *support, A, B);
        
        // check distance from origin to closestEdge against old min distance
		//std::cerr <<"old distance: " << old_distance <<", new distance: "<<edge.distance <<", difference: "<< edge.distance -old_distance << std::endl;
		//why the infinite loop?
        if ((iterations>10)||(edge.distance -old_distance  < _UTIL_GEOMETRY_EPSILON)) {
            return_penetration_vector = edge.normal;
            return_penetration_depth = edge.distance;
            return;
        } else {//continue to expand simplex
			//std::cerr <<"EPA new point: " << support->x <<","<<support->y <<","<< support->z << std::endl;
            std::vector<Util::Vector>::iterator iterator = simplex.begin();
            simplex.insert(simplex.begin()+edge.index, *support);
			old_distance=edge.distance;
        }
    }
}

bool decomposition(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB){
    
    std::vector<std::vector<Util::Vector>> trianglesA; std::vector<std::vector<Util::Vector>> trianglesB;
    
    double leftmostA = DBL_MAX; double leftmostB = DBL_MAX;
    int leftmostAIndex, leftmostBIndex;
    //get leftmost points in shapes A and B
    for(int i= 0; i<_shapeA.size(); i++){
        if(_shapeA.get(i).x < leftmostA){
            leftmostA = _shapeA.get(i).x;
            leftmostAIndex = i;
        }
    }
    
    for(int i= 0; i<_shapeB.size(); i++){
        if(_shapeA.get(i).x < leftmostA){
            leftmostA = _shapeB.get(i).x;
            leftmostBIndex = i;
        }
    }
    
    Util::Vector a1 = shapeA.get(leftmostAIndex); Util::Vector a2; Util::Vector a3; int nextPointIndex;
    if(leftmostAIndex==(shape.size()-1))
    {
        a2 = _shapeA.get(0);
        a3 = _shapeA.get(1);
        nextPointIndex = 2;
    }
    else if(leftmostAIndex==(shape.size()-2))
    {
        a2 = _shapeA.get(shape.size()-1);
        a3 = _shapeA.get(0);
        nextPointIndex = 1;
    }else
    {
        a2 = _shapeA.get(leftmostAIndex+1);
        a3 = _shapeA.get(leftmostAIndex+2);
        
        if(leftMostAIndex == (shape.size()-1))
            nextPointIndex = 0;
        else
            nextPointIndex = leftmostAIndex+2;
        
    }
    //int insideA = 1;
    //iterate through all non-triangle points, and check if they are within the current triangle
    for(int i=nextPointIndex; nextPointIndex!=leftmostAIndex; i++)
    {
        //checks to see if point is outside rectangle surrounding triangle
        double xMin = a1.x;
        double xMax = DBL_MIN; double yMin = DBL_MAX; double yMax = DBL_Min;
        
        if(a2.x >= a3.x)
            xMax = a2.x;
        
        if(a1.y >= a2.y)
        {
            yMax = a1.y;
            if(a3.y>=yMax)
                yMax = a3.y;
        }
        
        if(a1.y <= a2.y)
        {
            yMin = a1.y;
            if(a3.y<=yMin)
                yMin = a3.y;
        }
        
        if(_shapeA.get(i).x < xMin || _shapeA.get(i).x > xMax || _shapeA.get(i).y < yMin || _shapeA.get(i).y > yMax)
            continue;
        
        //checks to see if point is inside triangle
        else{
            //point is inside triangle
            if(sameSide(_shapeA.get(i), a1, a2, a3) && sameSide(_shapeA.get(i), a2, a1, a3) && sameSide(_shapeA.get(i), a3, a1, a2))
            {
            //TODO: set test triangle with an edge from the leftmost to the leftmost point inside of the triangle, I realize I have to keep iterating, but I don't know what the third vertex
            //in this triangle should be
            }
            else//store triangle
            {
                triangle = new std::vector<Util::Vector>; tVertices = new Util::Vector(a1,a2,a3);
                trianglesA.add(triangle);
            }
        }
        
        
    }
    //will be symmetric to A
    Util::Vector b1 = shapeB.get(leftmostBIndex); Util::Vector b2; Util::Vector b3;
    if(leftmostBIndex==(shape.size()-1))
    {
        b2 = _shapeB.get(0);
        b3 = _shapeA.get(1);
    }
    else if(leftmostAIndex==(_shapeB.size()-2))
    {
        b2 = _shapeB.get(_shapeB.size()-1);
        b3 = _shapeB.get(0);
    }
    else
    {
        b2 = _shapeB.get(leftmostBIndex+1);
        b3 = _shapeB.get(leftmostBIndex+2);
    }
    
    //get triangles from both shapes and test them for collisions pairwise
}

bool sameSide(Util::Vector p1, Util::Vector p2, Util::Vector a, Util::Vector b){
    if( ((p1.x - a.x) * (b.y - a.y) - (b.x - a.x) * (p1.y - a.y))
    * ((p2.x - a.x) * (b.y - a.y) - (b.x - a.x) * (p2.y - a.y)) > 0)
        return true;
    else
        return false
}





//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//TODO: pseudocode
	std::vector<Util::Vector> simplex;
	bool is_colliding = GJK(simplex,_shapeA,_shapeB);
	if (is_colliding == true)
	{
		EPA(_shapeA, _shapeB, simplex,return_penetration_depth, return_penetration_vector);
		return true;
	}
	else
	{
		return_penetration_depth=0;//can't return null to penetration vector because it's a reference
		return false;
	}
	
}