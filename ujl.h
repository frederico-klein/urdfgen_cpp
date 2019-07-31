#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "inc/tinyxml.h"

const double PI = 3.14159265359;

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

class UrdfRoot
{
public:
};

class OrVec
{
public:
	std::string xyz = "0 0 0";
	std::string rpy = "0 0 0";
	double x = 0, y = 0, z = 0, r = 0, p = 0, yaw = 0;
	bool isset = false;
	/*OrVec() {
	isset = false;
	xyz = "0 0 0";
	rpy = "0 0 0";
	};*/
	void setxyz(double xx, double yy, double zz);
	void setrpy(double rr, double pp, double yy);
};

class Visual
{
public:
	OrVec origin;
	std::string geometryfilename = "";
	std::string materialfilename = "";
	std::string color = "0.792156862745098 0.819607843137255 0.933333333333333 1";  // the colour that was being used in our other files.i am used to it, so i will keep it
};

class Collision
{
	OrVec origin;
	std::string geometryfilename = "";
};

class Inertia
{
	std::string ixx, ixy, ixz, iyy, iyz, izz;
};

class Inertial
{
public:
	OrVec origin;
	std::string mass;
	Inertia inertia;
};

class Limit
{
public:
	std::string lower = "-1";
	std::string upper = "1";
	std::string effort = "0";
	std::string velocity = "0";
};

class UJoint
{
public:
	//properties
	std::string name = "";
	std::string generatingjointname = "";
	OrVec origin; //SixDegree origin;
	OrVec realorigin;
	std::string parentlink;
	std::string childlink;
	std::string axis = "0 0 0";
	Limit limit;
	std::string type;
	bool isset = false;


	//perhaps unused?
	int row;
	int level;
	Ptr<Joint> entity;

	//methods
	UJoint() {};
	~UJoint() {};
	std::string setjoint(Ptr<Joint> joint); ////////// this is super incomplete as far as logging goes. we need a solution. either a shared object to collect errors (sounds more obnoxious than setting up logging, really...)
	void setrealorigin(OrVec);
	std::string getitems();
	void makexml(UrdfRoot);

	// when sixdegree is working!
	void setjoint(Ptr<Joint> joint, Ptr<CommandInput> cmdInput, Ptr<CommandInputs> inputs);

	//UJoint(std::string name_) 
	//{
	//	name = name_;
	//};
};


class ULink
{
public:
	//properties
	std::string name = "";
	Inertial inertial;
	Collision collision;
	OrVec coordinatesystem;
	bool isVirtual = true;
	std::vector<Occurrence> group; //vector of what?

	//what I think is unused
	int level;
	std::string parent;
	//row is defined in element and here, this is weird/unnecessary/wrong!
	int row;

	//methods
	std::string getitems();
	void genfatherjoint(UJoint joint);

private:
	//groupmembers
};



class UElement {
public:
	enum DATATYPE { DT_UNDEF, DT_JOINT, DT_LINK } type;
	union {
		UJoint joint;
		ULink link;
	};
	UElement() {
		type = DT_UNDEF;
	};
	UElement(UElement&) = default;
	void setElement(std::string eltype);
	~UElement() {};
	int row;
	//std::string name;
};
