#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "ujl.h"

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

typedef std::pair<int, UElement*> DicElement; //defines something like a python keyed dictionary

class UrdfTree
{
public:
	//properties
	
	std::vector<DicElement> elementsDict;
	UElement currentEl;

	// methods
	void addLink(std::string name, int row);
	void addJoint(std::string name, int row);
	void rmElement(int elnum);
	void genTree()
	{

		std::vector<DicElement> thiselementsdict = elementsDict;
		//std::vector<DicElement> thiselementsdict;
		//for some reason when I wrote this in python, deepcopy/copy didn't work, so I iterated over the dictionary and copied item per item
		//I do not remember why i needed to do this, but I suppose a shallow copy should work
		/*for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
		{

		}*/


	};
	void allLinks() ;
	void allJoints() ;
	void allElements() ;
	void getEl() ;
	void getCurrentElDesc() ;
	void setCurrentEl() ;
	UrdfTree() {};
	UrdfTree(UrdfTree&) = default;
	~UrdfTree() {};
private:
	std::pair<std::vector<DicElement>, std::vector<DicElement>> gentreefindbase(std::vector<DicElement> thiselementsdict);
	void gentreecore() ;
	void gentreecorecore() ;
	void genfatherjoint() ;
	void findjointscore() ;
	void findjoints() ;
	void alllinks() ;
	void alljoints();
};