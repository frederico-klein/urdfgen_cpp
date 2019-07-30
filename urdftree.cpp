#pragma once
#include "urdftree.h"
#include "ujl.h"

//// I can't have recursive references, so I actually need to use this bloody syntax

void UrdfTree::addLink(std::string name, int row)
{
	UElement thislink;
	thislink.setElement("link");
	thislink.link.name = name;
	thislink.row = row;
	DicElement thisElement = std::make_pair(row, &thislink);
	elementsDict.push_back(thisElement);
	//UElement {thislink(name);
};

void UrdfTree::addJoint(std::string name, int row)
{
	UElement thisjoint;
	thisjoint.setElement("joint");
	thisjoint.joint.name = name;
	thisjoint.row = row;
	DicElement thisElement = std::make_pair(row, &thisjoint);
	elementsDict.push_back(thisElement);
};

