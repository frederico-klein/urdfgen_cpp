#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "inc/tinyxml.h"
#include "inc/easylogging/easylogging++.h"
#include <filesystem>

const double PI = 3.14159265359;

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

namespace fs = std::filesystem;

//class UrdfRoot
//{
//public:
//};

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
public:
	OrVec origin;
	std::string geometryfilename = "";
};

class Inertia
{
public:
	std::string ixx, ixy, ixz, iyy, iyz, izz;
	void set(double, double, double, double, double, double);
};

class Inertial
{
public:
	OrVec origin;
	std::string mass = "0";
	Inertia inertia;
	void setall(double, double, double, double, double, double, double);
};

class Limit
{
public:
	std::string lower = "-1";
	std::string upper = "1";
	std::string effort = "0";
	std::string velocity = "0";
};

class UElement {
public:
	int row;
	std::string name;

	//perhaps unused
	int level;
	virtual std::string getitems() { return "not implemented"; };
	virtual void makexml(TiXmlElement*, std::string) 
	{
		LOG(DEBUG) << "UELement virtual makexml function was called!";
	};
	UElement() {};
	//UElement(UElement&) = default;
	~UElement() {};

	//std::string name;
};


class UJoint:public UElement
{
public:
	//properties

	std::string generatingjointname;
	OrVec origin; //SixDegree origin;
	OrVec realorigin;
	std::string parentlink;
	std::string childlink;
	std::string axis;
	Limit limit;
	std::string type;
	bool isset;

	//perhaps unused?
	int level;
	Ptr<Joint> entity;

	//methods
	UJoint() {
		name = "";
		generatingjointname = "";
		axis = "0 0 0";
		isset = false;
	};
	~UJoint() {};
	void setjoint(Ptr<Joint> joint); ////////// this is super incomplete as far as logging goes. we need a solution. either a shared object to collect errors (sounds more obnoxious than setting up logging, really...)
	void setrealorigin(OrVec);
	std::string getitems();
	void makexml(TiXmlElement*, std::string);

	// when sixdegree is working!
	void setjoint(Ptr<Joint> joint, Ptr<CommandInput> cmdInput, Ptr<CommandInputs> inputs);

	//UJoint(std::string name_) 
	//{
	//	name = name_;
	//};
};


class ULink:public UElement
{
public:
	//properties
	Inertial inertial;
	Visual visual;
	Collision collision;
	OrVec coordinatesystem;
	bool isVirtual = true;
	std::vector<Ptr<Occurrence>> group; //vector of what?
	Ptr<Joint> fatherjoint;
	//what I think is unused

	std::string parent;

	//methods
	std::string getitems();
	void genfatherjoint(UJoint joint);
	void makexml(TiXmlElement*, std::string);
	bool genlink(fs::path, fs::path, Ptr<Design>, Ptr<Application>);

private:
	//groupmembers
};