#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "ujl.h"

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

class UrdfTree
{
public:
	//properties
	typedef std::pair<int, UElement*> DicElement; //defines something like a python keyed dictionary
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
	//	void allLinks() {};
	//	void allJoints() {};
	//	void allElements() {};
	//	void getEl() {};
	//	void getCurrentElDesc() {};
	//	void setCurrentEl() {};
	UrdfTree() {};
	UrdfTree(UrdfTree&) = default;
	~UrdfTree() {};
private:
	std::pair<std::vector<DicElement>, std::vector<DicElement>> gentreefindbase(std::vector<DicElement> thiselementsdict) {
		bool foundbase = false;
		std::vector<DicElement> placedlinks;
		for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
		{
			//el is const_iterator
			//need to check if element is link
			if (el->second->type == UElement::DT_LINK && el->second->link.name == "base")
			{
				foundbase = true;

			}
		}
		return { placedlinks,thiselementsdict };
		//return std::make_tuple();
	};
	//	void gentreecore() {};
	//	void gentreecorecore() {};
	//	void genfatherjoint() {};
	//	void findjointscore() {};
	//	void findjoints() {};
	//	void alllinks() {};
	//	void alljoints() {};
};