#include "urdftree.h"
#include "ujl.h"

//// I can't have recursive references, so I actually need to use this bloody syntax

void UrdfTree::addLink(std::string name, int row)
{
	try {
		
		ULink thislink;
		thislink.name = name;
		thislink.row = row;
		ui->messageBox(name);
		DicElement thisElement = std::make_pair(row, &thislink);
		elementsDict.push_back(thisElement);

	}
	catch (...)
	{
		ui->messageBox("urdf::addlink failed!");
	};
};

void UrdfTree::addJoint(std::string name, int row)
{
	try {
		UJoint thisjoint;
		thisjoint.name = name;
		thisjoint.row = row;
		//ui->messageBox("this is okay");
		ui->messageBox(name);
		DicElement thisElement = std::make_pair(row, &thisjoint);
		elementsDict.push_back(thisElement);
	}
	catch (...)
	{
		ui->messageBox("urdf::addjoint failed!");
	};
};

void UrdfTree::allLinks() {};
void UrdfTree::allJoints() {};
std::string UrdfTree::allElements() 
{
	return "not implemented";
};
void UrdfTree::getEl() 
{
};
std::string UrdfTree::getCurrentElDesc() 
{
	return "not implemented";
};
void UrdfTree::setCurrentEl(int) {};


//private

std::pair<std::vector<DicElement>, std::vector<DicElement>> UrdfTree::gentreefindbase(std::vector<DicElement> thiselementsdict) {
	bool foundbase = false;
	std::vector<DicElement> placedlinks;
	for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
	{
		//el is const_iterator
		//need to check if element is link
		UElement* myel = el->second;
		ui->messageBox(myel->name);
		if (dynamic_cast<ULink*>(myel) && myel->name == "base")
			//if (el->second->type == UElement::DT_LINK && el->second->link.name == "base")
		{
			foundbase = true;
			ui->messageBox("hey found base!");

		}
	}
	return { placedlinks,thiselementsdict };
	//return std::make_tuple();
};
void  UrdfTree::gentreecore() {};
void  UrdfTree::gentreecorecore() {};
void  UrdfTree::genfatherjoint() {};
void  UrdfTree::findjointscore() {};
void  UrdfTree::findjoints() {};
void  UrdfTree::alllinks() {};
void  UrdfTree::alljoints() {};