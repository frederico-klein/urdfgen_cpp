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
	std::string containerPackage;
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

class ULink;

class UJoint:public UElement
{
public:
	//properties

	std::string generatingjointname;
	OrVec origin; //SixDegree origin;
	OrVec realorigin;
	ULink* parentlink;
	ULink* childlink;

	bool isFastSwitch;
	std::string parentPackage;
	std::string childPackage;

	std::string axis;
	Limit limit;
	std::string type;
	bool isset;

	Ptr<Joint> entity;

	//methods
	UJoint() {
		name = "";
		generatingjointname = "";
		axis = "0 0 0";
		isset = false;
		isFastSwitch = false;
	};
	~UJoint() {};
	void setjoint(Ptr<Joint> joint); 
	void setrealorigin(OrVec);
	std::string getitems();
	void makexml(TiXmlElement*, std::string);

	// when sixdegree is working!
	void setjoint(Ptr<Joint> joint, Ptr<CommandInput> cmdInput, Ptr<CommandInputs> inputs);
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
	bool isBase = false;
	std::vector<Ptr<Occurrence>> group; 
	Ptr<Joint> fatherjoint;
	std::string parent;
	//for srdf I also need to keep track of adjacent elements
	std::vector<ULink*> adjacentsList;
	std::vector<ULink*> selfAdjacents;
	std::vector<ULink*> mixedAdjacents;

	//methods
	std::string getitems();
	void genfatherjoint(UJoint joint);
	void makexml(TiXmlElement*, std::string);
	bool genlink(fs::path, fs::path, Ptr<Design>, Ptr<Application>);
	void addAdjacent(ULink*);
	void parseadjacents();
};