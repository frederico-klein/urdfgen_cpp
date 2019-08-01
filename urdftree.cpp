#include "urdftree.h"
#include "ujl.h"

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

void UrdfTree::genTree()
{

	std::vector<DicElement> placedlinks, thiselementsdict = elementsDict;
	TwoDic placed_and_this;
	//std::vector<DicElement> thiselementsdict;
	//for some reason when I wrote this in python, deepcopy/copy didn't work, so I iterated over the dictionary and copied item per item
	//I do not remember why i needed to do this, but I suppose a shallow copy should work
	/*for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
	{

	}*/
	//we don't have std::tie, so we need a bit more work here
	try
	{
		placed_and_this = gentreefindbase(thiselementsdict);
		placedlinks = placed_and_this.first;
		thiselementsdict = placed_and_this.second;
		DicElement firstel = placedlinks[0];
		ULink* firstlink = dynamic_cast<ULink*>(firstel.second);
		assert(firstlink->coordinatesystem.isset);
	}
	catch (...)
	{
		ui->messageBox("problems finding base!");
	}
	//now I need to find the joints that connect to base and place them alternatingly with the links!
	//danger part. a while. 
	int max_operations = 1000; //makes sure I break out of the while with an error and don't get stuck there forever
	int num_op = 0;
	bool still_things_to_be_placed = true;

	while (still_things_to_be_placed && num_op < max_operations)
	{
		std::pair<UJointList, TwoDic> joints_placed_this;
		joints_placed_this = findjoints(placed_and_this);
		//make sure we update our dictionaries so that things make sense. I suppose if I used pointers this would not be necessary
		UJointList placedjoints = joints_placed_this.first;
		placed_and_this = joints_placed_this.second;
		if (!placedjoints.empty()) 
		{

		}
	}

}

//private

TwoDic UrdfTree::gentreefindbase(std::vector<DicElement> thiselementsdict) {
	bool foundbase = false;
	std::vector<DicElement> placedlinks;
	for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
	{
		//el is const_iterator
		//need to check if element is link
		UElement* myel = el->second;
		ui->messageBox(myel->name);
		ULink* currLink = dynamic_cast<ULink*>(myel);
		if (currLink && myel->name == "base")
			//if (el->second->type == UElement::DT_LINK && el->second->link.name == "base")
		{
			foundbase = true;
			ui->messageBox("hey found base!");
			currLink->coordinatesystem.isset = true; //base coordinate system is zero by default, so it is already set
			DicElement mybase = std::make_pair(0,currLink);
			placedlinks.push_back(mybase);
			thiselementsdict.erase(thiselementsdict.begin() + el->first);
			break;
		}
	}
	if (!foundbase)
		ui->messageBox("did not find base!");
	return { placedlinks,thiselementsdict };
	//return std::make_tuple();
};
TwoDic UrdfTree::gentreecore(std::pair<UJointList, TwoDic> joints_placed_this) {};
void  UrdfTree::gentreecorecore() {};
void  UrdfTree::genfatherjoint() {};
void  UrdfTree::findjointscore() {};
std::pair<UJointList, TwoDic> UrdfTree::findjoints(TwoDic placed_and_this)
{


};
void  UrdfTree::alllinks() {};
void  UrdfTree::alljoints() {};