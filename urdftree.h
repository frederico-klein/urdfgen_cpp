#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "ujl.h"

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

typedef std::pair<int, UElement*> DicElement; //defines something like a python keyed dictionary
typedef std::pair<std::vector<DicElement>, std::vector<DicElement>> TwoDic;
typedef std::vector<UJoint> UJointList;

class UrdfTree
{
public:
	//properties
	
	std::vector<DicElement> elementsDict;
	UElement* currentEl;
	Ptr<UserInterface> ui; // I kinda want to debug this.

	// methods
	void addLink(std::string name, int row);
	void addJoint(std::string name, int row);
	void rmElement(int elnum);
	void genTree();
	void allLinks() ;
	void allJoints() ;
	std::string allElements() ; //this is actually another custom std::pair :(
	void getEl() ;
	std::string getCurrentElDesc() ;
	void setCurrentEl(int) ;
	UrdfTree() {};
	//UrdfTree(Ptr<UserInterface> ui_) { ui = ui_; };
	UrdfTree(UrdfTree&) = default;
	~UrdfTree() {};
private:
	TwoDic gentreefindbase(std::vector<DicElement>);
	TwoDic gentreecore(std::pair<UJointList, TwoDic>) ;
	void gentreecorecore(std::pair<UJointList, TwoDic>, UJoint) ;
	void genfatherjoint() ;
	void findjointscore() ;
	std::pair<UJointList,TwoDic> findjoints(TwoDic) ;
	void alllinks() ;
	void alljoints();
};