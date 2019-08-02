#include "urdftree.h"
#include "ujl.h"

void UrdfTree::addLink(std::string name, int row)
{
	try {
		
		ULink* thislink = new ULink();
		thislink->name = name;
		thislink->row = row;
		//remove!
		ui->messageBox(name);
		DicElement thisElement = std::make_pair(row, thislink);
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
		UJoint* thisjoint = new UJoint();
		thisjoint->name = name;
		thisjoint->row = row;
		//remove!
		ui->messageBox(name);
		DicElement thisElement = std::make_pair(row, thisjoint);
		elementsDict.push_back(thisElement);
	}
	catch (...)
	{
		ui->messageBox("urdf::addjoint failed!");
	};
};

UElement* UrdfTree::getEl(int i) 
{
	//ui->messageBox("UrdfTree::getEl reached");
	UElement* thisEl;

	for (auto el : elementsDict) // I need this because the list index does not correspont to how many items are there in the dic. 
	{
		//ui->messageBox(std::to_string(el.first));
		if (el.first == i)
		{
			//i don't like this, lets try it the simpler way
			//thisEl = dynamic_cast<UElement*>(el.second);
			thisEl = el.second;
			//ui->messageBox("found element!"+thisEl->name);
			break;
		}
	}
	//ui->messageBox("UrdfTree::getEl ended");
	return thisEl;

};
std::string UrdfTree::getCurrentElDesc() 
{
	if (currentEl)
		return currentEl->name + "\n" + currentEl->getitems();
	else
		return "no current element";
};
void UrdfTree::setCurrentEl(int i) 
{
	//ui->messageBox("UrdfTree::setCurrentEl reached");
	UElement* thisEl = getEl(i);
	if (thisEl)
	{
		currentEl = thisEl;
		//ui->messageBox("current element set to" + currentEl->name);
	}
	//ui->messageBox("UrdfTree::setCurrentEl ended");

};
pair<string, ULinkList> UrdfTree::allLinks()
{
	pair<pair<string, ULinkList>, vector<string>> alllinksout = alllinks(elementsDict);
	return alllinksout.first;
};
pair<string, UJointList> UrdfTree::allJoints()
{
	pair<pair<string, UJointList>, vector<string>> alljointsout = alljoints(elementsDict);
	return alljointsout.first;
};
pair<string, vector<UElement*>> UrdfTree::allElements()
{
	string exstr;
	bool noels = true;
	vector<UElement*> allels;
	vector<string> allelnames;

	for (auto el : elementsDict)
	{
		std::string namename = "";
		ULink* currLink = dynamic_cast<ULink*>(el.second);
		if (currLink)
		{
			namename = "link: " + currLink->name+"\n";
		}
		UJoint* currJoint = dynamic_cast<UJoint*>(el.second);
		if (currJoint)
		{
			namename = "joint: " + currJoint->name + "\n";
		}
		if (!currLink && !currJoint)
		{
			namename = "unk!\n";
		}
		exstr = exstr + namename;
		allels.push_back(el.second);
		noels = false;
	}
	if (noels)
		exstr = "no elements!";
	//ui->messageBox("not implemented");

	return make_pair(exstr,allels);
};
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
		num_op++;
		std::pair<UJointList, TwoDic> joints_placed_this;
		joints_placed_this = findjoints(placed_and_this);
		//make sure we update our dictionaries so that things make sense. I suppose if I used pointers this would not be necessary
		UJointList placedjoints = joints_placed_this.first;
		placed_and_this = joints_placed_this.second;
		if (!placedjoints.empty()) 
		{
			placed_and_this = gentreecore(joints_placed_this); //this is bad. at some point I will forget to update these and the code will not work. consider change to pointers!
			placedlinks = placed_and_this.first;
			thiselementsdict = placed_and_this.second;
		}
		else
		{
			still_things_to_be_placed = false;
			if (!thiselementsdict.empty())
			{
				//
				ui->messageBox(" floating elements found. This tree is not correct, please review your work!");
			}
		}
	}
	if (num_op == max_operations)
	{
		ui->messageBox("reached maximum number of operations. unexpected. check code!");
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
TwoDic UrdfTree::gentreecore(std::pair<UJointList, TwoDic> joints_placed_this) 
{
	TwoDic placed_and_this = joints_placed_this.second;
	//for (auto it = joints_placed_this.first.cbegin(); it != joints_placed_this.first.cend(); it++)
	for (UJoint* joint :joints_placed_this.first)		
	{
		bool* stillmerging = new bool;
		*stillmerging = true;
		while (&stillmerging)
		{
			placed_and_this = gentreecorecore(placed_and_this ,joint, stillmerging);
		}

	}
	return placed_and_this;
};
TwoDic UrdfTree::gentreecorecore(TwoDic placed_and_this, UJoint* joint, bool* stillmerging)
{
	//unpacking...
	//actually, not using pointers here will have also affect speed
	//UJointList placedjoints = joints_placed_this.first;
	//TwoDic placed_and_this = joints_placed_this.second;
	std::vector<DicElement> placedeldic = placed_and_this.first;
	std::vector<DicElement> thiseldic = placed_and_this.second;
	
	*stillmerging = false;
	for (DicElement el: thiseldic)
	{
		ULink* currLink = dynamic_cast<ULink*>(el.second);
		if (currLink && currLink->name == joint->childlink)
		{
			DicElement placeel = std::make_pair(placedeldic.size(), currLink);
			placedeldic.push_back(placeel);
			genfatherjoint(el.second->name, joint);
			*stillmerging = true;
			thiseldic.erase(thiseldic.begin() + el.first); //I changed this a bit, hopefully it doesn't break
			break;
		}
	}
	return std::make_pair(placedeldic, thiseldic);

};
void  UrdfTree::genfatherjoint(std::string name_, UJoint* joint) 
{
	for (DicElement dicEl : elementsDict)
	{
		ULink* currLink = dynamic_cast<ULink*>(dicEl.second);
		if (currLink && currLink->name == name_) //this should be a link, perhaps I should check as well, in case i have a joint and a link with the same name!
			currLink->genfatherjoint(*joint);			
	}
};
DicElement UrdfTree::findjointscore(TwoDic placed_and_this) 
{
	//returns a joint that can be placed

	//comments from python script:
	// here is the place to look for whether parent and child are flipped. 
	// this is not done, I will assume the person creating the model has checked this!
	// i can also check for closed loops as well (but that would be harder...)

	pair<pair<string, ULinkList>, vector<string>> alllinksout = alllinks(elementsDict); // I might not need this
	vector<string> allplacedlinks = alllinksout.second; // I might not need this

	UJoint* myjoint;

	int el_row;
	//unpacking twidic
	vector<DicElement> placedeldic, thiselementsdict;

	for (DicElement el : thiselementsdict)
	{
		myjoint = dynamic_cast<UJoint*>(el.second);
		if (myjoint)
		{
			el_row = el.first;
			//check if joint's parent is in allplacedlinks
			if (std::find(allplacedlinks.begin(), allplacedlinks.end(), myjoint->parentlink) != allplacedlinks.end()) // I might not need this
			{
				// Element in vector.
				// This feels very awkward, I think I am searching twice
				for (auto elel : placedeldic)
				{
					ULink* mylink = dynamic_cast<ULink*>(elel.second);
					if (mylink && mylink->name == myjoint->parentlink)
					{
						//found a link of which I can have the real coordinate system
						//that is, up to this part in the chain, all the offsets are accounted for.
						assert(mylink->coordinatesystem.isset);
						myjoint->setrealorigin(mylink->coordinatesystem);
					}
				}
				break;
			}
		}
	}
	DicElement myJointElement = make_pair(el_row,myjoint);
	return myJointElement;
};
std::pair<UJointList, TwoDic> UrdfTree::findjoints(TwoDic placed_and_this)
{
	//finds all the joints that can be placed, i.e., whose parent links are already placed
	bool madamada = true;
	UJointList foundjoints;
	//unpacking
	std::vector<DicElement> placedelements = placed_and_this.first, thiselementsdict = placed_and_this.second;
	while (madamada)
	{
		DicElement jointDicElement = findjointscore(placed_and_this);
		//casting...
		UJoint* joint = dynamic_cast<UJoint*>(jointDicElement.second);

		if (joint)
		{
			foundjoints.push_back(joint);
			thiselementsdict.erase(thiselementsdict.begin() + jointDicElement.first);
			DicElement DEJoint = make_pair(placedelements.size(),joint);
			placedelements.push_back(DEJoint);
		}
		else
		{
			madamada = false;
		}
	}
	//packing
	TwoDic placed_and_this_out = make_pair(placedelements, thiselementsdict);
	return make_pair(foundjoints,placed_and_this_out);

};
pair<pair<string, ULinkList>, vector<string>> UrdfTree::alllinks(vector<DicElement> someeldic) {
	string exstr;
	bool nolinks = true;
	ULinkList alllinks;
	vector<string> alllinknames;

	for (auto el : elementsDict)
	{
		ULink* currLink = dynamic_cast<ULink*>(el.second);
		if (currLink)
		{
			exstr = exstr + "link: " + currLink->name + "\n";
			alllinks.push_back(currLink);
			alllinknames.push_back(currLink->name);
			nolinks = false;
		}
	}
	if (nolinks)
		exstr = "no links!";

	return make_pair(make_pair(exstr, alllinks), alllinknames); //hmm...

};
pair<pair<string, UJointList>, vector<string>> UrdfTree::alljoints(vector<DicElement> someeldic) {
	string exstr;
	bool nojoints = true;
	UJointList alljoints;
	vector<string> alljointnames;

	for (auto el : elementsDict)
	{
		UJoint* currJoint = dynamic_cast<UJoint*>(el.second);
		if (currJoint)
		{
			exstr = exstr + "joint: " + currJoint->name + "\n";
			alljoints.push_back(currJoint);
			alljointnames.push_back(currJoint->name);
			nojoints = false;
		}
	}
	if (nojoints)
		exstr = "no joints!";
	
	return make_pair(make_pair(exstr, alljoints), alljointnames);

};

