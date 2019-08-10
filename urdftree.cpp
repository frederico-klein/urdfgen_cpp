#include "urdftree.h"
#include "ujl.h"
#include "inc/easylogging/easylogging++.h"
#include "shared_funcs.h"

const int MAX_OP = 10;

void UrdfTree::removeElementByName(vector<DicElement>* thiselementsdict, string name)
{
	size_t i;
	bool thisshouldprobablybeamethodfromadictionaryclass_foundmyelement = false;
	for (i = 0; i < thiselementsdict->size(); i++)
	{
		if (name == (*thiselementsdict)[i].second->name)
		{
			thisshouldprobablybeamethodfromadictionaryclass_foundmyelement = true;
			break;
		}
	}
	assert(thisshouldprobablybeamethodfromadictionaryclass_foundmyelement);
	LOG(INFO) << "trying to remove item:" + std::to_string(i) + "\nelement i think i am removing:" + (*thiselementsdict)[i].second->name;
	thiselementsdict->erase(thiselementsdict->begin() + i);

};

void UrdfTree::addLink(std::string name, int row)
{
	try {
		
		ULink* thislink = new ULink();
		thislink->name = name;
		thislink->row = row;
		LOG(DEBUG)<< "added link with name: " + name;
		DicElement thisElement = std::make_pair(row, thislink);
		elementsDict.push_back(thisElement);

	}
	catch (...)
	{
		std::string errormsg = "urdf::addlink failed!";
		LOG(ERROR) << errormsg;
		ui->messageBox(errormsg);
	};
};
void UrdfTree::addJoint(std::string name, int row)
{
	try {
		UJoint* thisjoint = new UJoint();
		thisjoint->name = name;
		thisjoint->row = row;
		thisjoint->childlink = "nonameyet"+name;
		thisjoint->parentlink = "nonameyet"+name;
		LOG(DEBUG) << "added joint with name: " + name;
		DicElement thisElement = std::make_pair(row, thisjoint);
		elementsDict.push_back(thisElement);
	}
	catch (...)
	{
		std::string errormsg = "urdf::addjoint failed!";
		LOG(ERROR) << errormsg;
		ui->messageBox(errormsg);
	};
};
UElement* UrdfTree::getEl(int i) 
{
	LOG(DEBUG) << "UrdfTree::getEl reached";
	UElement* thisEl{0};

	for (auto el : elementsDict) // I need this because the list index does not correspont to how many items are there in the dic. 
	{
		if (el.first == i)
		{
			thisEl = el.second;
			break;
		}
	}
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
	LOG(DEBUG) << "UrdfTree::setCurrentEl reached";
	UElement* thisEl = getEl(i);
	if (thisEl)
	{
		currentEl = thisEl;
		LOG(DEBUG) << "current element set to" + currentEl->name;
	}
	// if I set i to -1, I get a null pointer
	if (i == -1)
	{
		LOG(DEBUG) << "setting current element to null";
		UElement* nullEl{ 0 };
		currentEl = nullEl;
	}
	LOG(DEBUG) << "UrdfTree::setCurrentEl ended";

};
pair<string, ULinkList> UrdfTree::allLinks()
{
	pair<pair<string, ULinkList>, vector<string>> alllinksout = alllinks(elementsDict);
	return alllinksout.first;
};
vector<string> UrdfTree::allLinksvec()
{
	pair<pair<string, ULinkList>, vector<string>> alllinksout = alllinks(elementsDict);
	return alllinksout.second;
};
pair<string, UJointList> UrdfTree::allJoints()
{
	pair<pair<string, UJointList>, vector<string>> alljointsout = alljoints(elementsDict);
	return alljointsout.first;
};
pair<string, vector<UElement*>> UrdfTree::allElements()
{
	pair<pair<string, vector<UElement*>>, vector<string>> allelssout = allelements(elementsDict);
	return allelssout.first;
};

std::string UrdfTree::genTree()
{
	
	LOG(INFO) << bigprint("starting gentree");

	report = "Report:\n";
	std::vector<DicElement> placedlinks, thiselementsdict = elementsDict;
	TwoDic placed_and_this;

	try
	{
		placed_and_this = gentreefindbase(thiselementsdict);
		placedlinks = placed_and_this.first;
		thiselementsdict = placed_and_this.second;
		DicElement firstel = placedlinks[0];
		ULink* firstlink = dynamic_cast<ULink*>(firstel.second);
		if (!firstlink->coordinatesystem.isset)
			throw "Coordinate system for the base not set! Resulting model would be incorrect";
	}
	catch (char* msg)
	{
		report += string(msg);
		LOG(ERROR) << msg;
		ui->messageBox(string(msg)+"\nproblems finding base!\nCannot proceed.");
		return report;
	}
	catch (...)
	{
		ui->messageBox("problems finding base!");
	}
	//now I need to find the joints that connect to base and place them alternatingly with the links!

	int max_operations = MAX_OP; 
	int num_op = 0;
	bool still_things_to_be_placed = true;

	while (still_things_to_be_placed && num_op < max_operations)
	{
		LOG(DEBUG) << "reached while loop";
		num_op++;
		std::pair<UJointList, TwoDic> joints_placed_this;
		joints_placed_this = findjoints(placed_and_this);
		LOG(DEBUG) << "findjoints seemed to be okay";
		
		UJointList placedjoints = joints_placed_this.first;
		placed_and_this = joints_placed_this.second;
		if (!placedjoints.empty()) 
		{
			LOG(DEBUG) << "!placedjoints.empty() , that means I have links to place!";
			placed_and_this = gentreecore(joints_placed_this); 
			LOG(DEBUG) << "gentreecore seemed to have worked";
			placedlinks = placed_and_this.first;
			thiselementsdict = placed_and_this.second;

		}
		else
		{
			still_things_to_be_placed = false;
			if (!thiselementsdict.empty())
			{
				//todo:perhaps show what elements those are?
				report += " floating elements found. This tree is not correct, please review your work!";
				return report;
			}
		}
	}
	if (num_op == max_operations)
	{
		report += "reached maximum number of operations. unexpected. check code!";
	}
	LOG(INFO) << report;
	LOG(INFO) << bigprint(" finishing gentree ");
	return report;
}
void UrdfTree::rmElement(int elnum)
{
	elementsDict.erase(elementsDict.begin() + elnum);
};
std::string UrdfTree::getdebugtext()
{
	std::pair<string, vector<UElement*>> alllinkstrpair = allElements();

	return "current element: " + getCurrentElDesc() + "\n" + alllinkstrpair.first;

};
//private
TwoDic UrdfTree::gentreefindbase(std::vector<DicElement> thiselementsdict) {
	bool foundbase = false;
	std::vector<DicElement> placedlinks;
	for (auto el = elementsDict.cbegin(); el != elementsDict.cend(); el++)
	{
		//el is const_iterator
		//need to check if element is link
		UElement* myel = el->second;

		ULink* currLink = dynamic_cast<ULink*>(myel);
		if (currLink && myel->name == "base")

		{
			foundbase = true;
			LOG(DEBUG) << "hey found base!";
			report += "found my base when testing. base is on row:" + std::to_string(currLink->row) + "\n";
			currLink->coordinatesystem.isset = true; //base coordinate system is zero by default, so it is already set
			currLink->isBase = true;
			DicElement mybase = std::make_pair(0,currLink);
			placedlinks.push_back(mybase);
			thiselementsdict.erase(thiselementsdict.begin() + el->first);
			break;
		}
	}
	if (!foundbase)
		ui->messageBox("did not find base!");
	return { placedlinks,thiselementsdict };

};
TwoDic UrdfTree::gentreecore(std::pair<UJointList, TwoDic> joints_placed_this) 
{
	int max_op = MAX_OP, num_op=0;
	TwoDic placed_and_this = joints_placed_this.second;

	std::string thisjointstr = alljoints(joints_placed_this.first);
	LOG(DEBUG) << "all placed(?) joints" + thisjointstr;

	for (UJoint* joint :joints_placed_this.first)		
	{
		bool stillmerging = true;
		while (stillmerging&&num_op<max_op)
		{
			num_op++;
			placed_and_this = gentreecorecore(placed_and_this ,joint, &stillmerging);
		}

	}
	assert(num_op<max_op);
	return placed_and_this;
};
TwoDic UrdfTree::gentreecorecore(TwoDic placed_and_this, UJoint* joint, bool* stillmerging)
{
	std::vector<DicElement> placedeldic = placed_and_this.first;
	std::vector<DicElement> thiseldic = placed_and_this.second;
	
	*stillmerging = false;
	for (DicElement el: thiseldic)
	{
		ULink* currLink = dynamic_cast<ULink*>(el.second);

		if (currLink)
		{
			LOG(DEBUG) << "current element is a link\ncurrentelement has name"+ currLink->name +"\ncurrent joint childlink is" + joint->childlink ;
		}

		if (currLink && currLink->name == joint->childlink)
		{
			DicElement placeel = std::make_pair(placedeldic.size(), currLink);
			placedeldic.push_back(placeel);
			genfatherjoint(el.second->name, joint);
			*stillmerging = true;
			removeElementByName(&thiseldic, currLink->name);

			report += "placed a link named:" + el.second->name + " because joint named:" + joint->name + "told me to!\n";
			LOG(DEBUG) << "report so far:\n" + report;
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
		if (currLink && currLink->name == name_) 
			currLink->genfatherjoint(*joint);			
	}
};
DicElement UrdfTree::findjointscore(vector<DicElement>* placedeldic, vector<DicElement>* thiselementsdict)
{
	//returns a joint that can be placed

	//comments from python script:
	// here is the place to look for whether parent and child are flipped. 
	// this is not done, I will assume the person creating the model has checked this!
	// i can also check for closed loops as well (but that would be harder...)
	pair<pair<string, vector<UElement*>>, vector<string>> allelsout1 = allelements(*thiselementsdict);
	string allremainingelements = allelsout1.first.first;
	pair<pair<string, vector<UElement*>>, vector<string>> allelsout2 = allelements(*placedeldic);
	string allplacedelements = allelsout2.first.first;
	LOG(DEBUG) << "findjointscore: placed elements\n"+ allplacedelements+"\nremaining elements"+ allremainingelements;



	UJoint* myjoint = new UJoint();

	bool foundjoint = false;
	int el_row;
	LOG(DEBUG) << "findjointscore initialized all vars okay.";
	LOG(DEBUG) << "IMPORTANT:::findjointscore thiselementsdict size is "+std::to_string(thiselementsdict->size());

	for (DicElement el : *thiselementsdict)
	{
		myjoint = dynamic_cast<UJoint*>(el.second);
		LOG(DEBUG) << "findjointscore dyncast okay!";
		if (myjoint)
		{			
			LOG(DEBUG) << "findjointscore found an element that is a joint!";
			el_row = el.first;
				for (auto elel : *placedeldic)
				{
					ULink* mylink = dynamic_cast<ULink*>(elel.second);
					if (mylink && mylink->name == myjoint->parentlink)
					{
						foundjoint = true;
						//found a link of which I can have the real coordinate system
						//that is, up to this part in the chain, all the offsets are accounted for.
						LOG(DEBUG) << "findjointscore::so far so good. checking assertion";
						if (mylink->coordinatesystem.isset)
							LOG(DEBUG) << "findjointscore::so far so good. okay to proceed";
						else
							LOG(DEBUG) << "findjointscore:: no good. ASSERT WILL FAIL!";


						if (!mylink->coordinatesystem.isset)
							LOG(ERROR) << "Coordinate system is not set! the Link's origin will be incorrect and the resulting model will need to be fixed manually!";
						myjoint->setrealorigin(mylink->coordinatesystem);
					}
				}
				break;
		}
	}

	if (!foundjoint)
		myjoint = nullptr;
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
	DicElement jointDicElement;

	while (madamada)
	{
		//setting debug vars!{
		pair<pair<string, vector<UElement*>>, vector<string>> allelsout1 = allelements(thiselementsdict);
		string allremainingelements = allelsout1.first.first;
		pair<pair<string, vector<UElement*>>, vector<string>> allelsout2 = allelements(placedelements);
		string allplacedelements = allelsout2.first.first;
		
		LOG(DEBUG) << "findjoint: size of thiselementsdict:" +std::to_string(thiselementsdict.size())+"\nplaced elements\n" + allplacedelements + "\nremaining elements" + allremainingelements;

		//}end debug things!
		jointDicElement = findjointscore(&placedelements, &thiselementsdict);
		//casting...
		UJoint* joint = dynamic_cast<UJoint*>(jointDicElement.second);

		if (joint)
		{
			LOG(INFO) << "findjoint: found a joint even!\njointname:"+joint->name;

			foundjoints.push_back(joint);
			
			removeElementByName(&thiselementsdict, joint->name);
					   			 
			DicElement DEJoint = make_pair(placedelements.size(),joint);
			placedelements.push_back(DEJoint);
			report += "placed joint:" + joint->name + "\n";
			LOG(INFO) << "report so far:\n" + report;
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

	return make_pair(make_pair(exstr, alllinks), alllinknames); 

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
string UrdfTree::alljoints(UJointList someujlist) {
	string exstr;
	bool nojoints = true;

	for (auto el : someujlist)
	{
		exstr = exstr + "joint: " + el->name + "\n";		
		nojoints = false;
	}
	if (nojoints)
		exstr = "no joints!";

	return exstr;
};
pair<pair<string, vector<UElement*>>, vector<string>> UrdfTree::allelements(vector<DicElement> thiselementsdict)
{
	string exstr;
	bool noels = true;
	vector<UElement*> allels;
	vector<string> allelnames;

	for (auto el : thiselementsdict)
	{
		std::string namename = "";
		ULink* currLink = dynamic_cast<ULink*>(el.second);
		if (currLink)
		{
			namename = "link: " + currLink->name + "\n";
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
		allelnames.push_back(namename);
		noels = false;
	}
	if (noels)
		exstr = "no elements!";

	return make_pair(make_pair(exstr, allels), allelnames);
};