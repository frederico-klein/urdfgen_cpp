#include "urdftree.h"
#include "ujl.h"
#include "inc/easylogging/easylogging++.h"
#include "shared_funcs.h"

namespace fs = std::filesystem;

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
		ULink* nolink = new ULink();
		nolink->name = "nonameyet" + name;
		thisjoint->childlink = nolink;
		thisjoint->parentlink = nolink;
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
UElement* UrdfTree::getElementByName(std::string name_)
{
	LOG(DEBUG) << "UrdfTree::getElementByName reached";
	UElement* thisEl{ 0 };

	for (auto el : elementsDict) // I need this because the list index does not correspont to how many items are there in the dic. 
	{
		if (el.second->name == name_)
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
	LOG(INFO) << bigprint(" finishing gentree basic");

	// now let's distribute it into multiple packages!
	bool isAllGood = true;
	for (auto el : elementsDict)
	{
		bool foundPackage = false;
		//for (auto package : packageTree)
		for (auto package = packageTree.begin(); package != packageTree.end(); package++) //we want to modify this, so no constant iterators... otherwise, what's the point?
		{
			if (package->name == el.second->containerPackage)
			{
				LOG(INFO) << "found package for element: " +el.second->name << "package is:" + package->name;
				foundPackage = true;
				package->elementsDict.push_back(el);
			}

			if (foundPackage)
				break;
		}
		if (!foundPackage)
		{
			isAllGood = false;
			report += "could not find package" + el.second->containerPackage +"\nGenerating this model will fail!";
		}
	}
	if (isAllGood)
		report += "Splitting elements to packages successful. This tree can be distributed in subpackages!";

	//now we also need to set the fsFatherLink so we can create views appropriately

	for (auto package = packageTree.begin(); package != packageTree.end(); package++) //we want to modify this, so no constant iterators... otherwise, what's the point?
	{
		bool fsFatherLinkisset = false;
		for (auto el : package->elementsDict)
		{
			//find the generating fsJoint. I am assuming only one per package. if we have a star with a bunch of fss, this will fail
			UJoint* myFsJoint = dynamic_cast<UJoint*>(el.second);
			if (myFsJoint && myFsJoint->isFastSwitch)
			{
				//okay found it. 
				package->fsFatherLink = myFsJoint->parentlink;
				fsFatherLinkisset = true;
			}
		}
		if (!fsFatherLinkisset)
		{
			LOG(WARNING) << "fsFatherLink not set!. will fail to generate view. ";
			//lets try to set this to base
			package->fsFatherLink = dynamic_cast<ULink*>(getElementByName("base"));
			if (!package->fsFatherLink)
				LOG(ERROR) << "attempt to use base as a link did not work. ";
		}
	}

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
			//will also set base to be in the first package!
			currLink->containerPackage = packageTree[0].name; //TODO: all the references should point to the object not the name, so it can be renamed!

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
			LOG(DEBUG) << "current element is a link\ncurrentelement has name"+ currLink->name +"\ncurrent joint childlink is" + joint->childlink->name ;
		}

		if (currLink && currLink->name == joint->childlink->name)
		{
			DicElement placeel = std::make_pair(placedeldic.size(), currLink);
			placedeldic.push_back(placeel);
			genfatherjoint(el.second->name, joint);
			*stillmerging = true;
			removeElementByName(&thiseldic, currLink->name);

			report += "placed a link named:" + el.second->name + " because joint named:" + joint->name + "told me to!\n";
			//setting up containerPackage
			if (joint->containerPackage == "") 
			{
				LOG(ERROR) << joint->name << " does not have containerPackage set! we messed up";
			}
			assert(joint->containerPackage != ""); 
			currLink->containerPackage = joint->containerPackage;
			report += "containerPackage: " + currLink->containerPackage + "\n";
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
					if (mylink && mylink->name == myjoint->parentlink->name)
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

						//for srdf I also need to keep track of adjacent elements



						//now if this joint is an FS joint, I will need to change the package context. if it isn't it keeps the context. links just keep the context of their father joints!

						if (myjoint->isFastSwitch)
						{
							//need to change the package to the next one in the list
							//theoretically, if I don't screw up, this should also place closed chains with 2 FSs, but it is hard to guarantee we will not have unexpected behaviour with more complex linkage chains
							//I mean, if there is a mistake and the fs joined by one side is defined on one package and from the other side defined to another, this procedure will break them apart. it will be up to the user to make sure things are correct.
							//I could do an extra flood-fill like algorithm to check this, but this would be rather hard and I don't see the point in doing so now. 
							myjoint->containerPackage = myjoint->childPackage;

							//eh, not asserts...
							assert(myjoint->childPackage != "");
							assert(myjoint->parentPackage != "");
						}
						else 
						{
							LOG(DEBUG) << "setting joint's container package, since it isn't FS joint to:" << mylink->containerPackage;
							myjoint->containerPackage = mylink->containerPackage;
							myjoint->parentPackage = mylink->containerPackage;
							myjoint->childPackage = mylink->containerPackage;
						}

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

vector<std::string> UrdfTree::packageList()
{
	vector<std::string> mylist;
	for (auto package : packageTree)
	{
		mylist.push_back(package.name);
	}
	return mylist;
};


//UPackage stuff::
void UPackage::makeView()
{
	LOG(INFO) << "Generating view Xacro for package " << name;
	TiXmlDocument thisurdfdoc;

	//xacro view part
	{
		TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
		thisurdfdoc.LinkEndChild(decl);

		TiXmlElement * thisurdfdocrobot_root = new TiXmlElement("robot");
		thisurdfdocrobot_root->SetAttribute("name", "gummi");
		thisurdfdoc.LinkEndChild(thisurdfdocrobot_root);

		TiXmlComment* comment = new TiXmlComment();
		comment->SetValue(" Some comment.");
		thisurdfdoc.LinkEndChild(comment);

		ULink base_link;
		base_link.name = "base_link";

		//maybe the best thing here is to make this an xacro parameter and then set the parameter to fsFatherLink
		ULink virtual_base_link;
		LOG(INFO) << "maybe I cant set fsFatherLink?";
		if (!fsFatherLink)
		{
			LOG(ERROR) << "fatherlink not set view cannot be made!";
			return;
		}
		LOG(INFO) << fsFatherLink->name;
		virtual_base_link.name = fsFatherLink->name; //this needs to be set!
		LOG(INFO) << "fsFatherLink was set okay";

		base_link.makexml(thisurdfdocrobot_root, name);
#
		UJoint setaxisjoint;
		setaxisjoint.name = "set_worldaxis";
		setaxisjoint.isset = true;
		setaxisjoint.type = "fixed";
		setaxisjoint.realorigin.rpy = std::to_string(PI / 2) + " 0 0";
		setaxisjoint.parentlink = &base_link;
		setaxisjoint.childlink = &virtual_base_link; 
		//setaxisjoint.childlink = dynamic_cast<ULink*>(_ms.thistree.getElementByName("base")); ////////////not the base, but the father of the FS joint!
		setaxisjoint.makexml(thisurdfdocrobot_root, name);

		TiXmlElement * thisurdfxacromacro = new TiXmlElement("xacro:include");
		thisurdfxacromacro->SetAttribute("filename", ("$(arg " + name + "_dir)/xacro/thissegment.urdf.xacro").c_str());
		thisurdfdocrobot_root->LinkEndChild(thisurdfxacromacro);

		// we need some more things to generate the view for this link


		string thissegmentxacroname_view = ("view_thissegment.urdf.xacro");
		string filenametosave_view = (xacro_directory / thissegmentxacroname_view).string();

		LOG(INFO) << "Saving view file: " + (filenametosave_view);
		thisurdfdoc.SaveFile(filenametosave_view.c_str());
	}

};

void UPackage::setpath(fs::path thisscriptpath, fs::path basemost_directory)
{
	try {
		LOG(DEBUG) << "called setpath";

		LOG(INFO) << "This script's path: " + (thisscriptpath.string());

		LOG(INFO) << "Basemost directory:" + basemost_directory.string();
		base_directory = basemost_directory / name;

		LOG(INFO) << "Base directory for this package:" + base_directory.string();
		if (!fs::exists(base_directory))
			fs::create_directories(base_directory); //// will create whole tree if needed
		meshes_directory		= base_directory / "meshes";
		components_directory	= base_directory / "components";
		xacro_directory			= base_directory / "xacro";
		config_directory		= base_directory / "config";


		if (!fs::exists(meshes_directory))
			fs::create_directory(meshes_directory);
		if (!fs::exists(components_directory))
			fs::create_directory(components_directory);
		if (!fs::exists(xacro_directory))
			fs::create_directory(xacro_directory);
		if (!fs::exists(config_directory))
			fs::create_directory(config_directory);


		vector<string> filestochange = { "display.launch", "urdf_.rviz", "package.xml", "CMakeLists.txt" };

		for (auto myfilename : filestochange)
		{
			ifstream file_in;
			// Read in the file

			auto thisfilename = thisscriptpath / "resources" / "subchain" / myfilename;

			LOG(INFO) << "Opening file:" + (thisfilename.string());
			file_in.open(thisfilename);
			if (!file_in)
				LOG(ERROR) << "failed to open input file:" + thisfilename.string();

			auto thisoutfilename = base_directory / myfilename;
			ofstream file_out(thisoutfilename.string(), std::ofstream::out);

			string filedata;

			while (std::getline(file_in, filedata))
			{
				// Replace the target string

				replaceAll(filedata, "subchain_template", name);

				// Write the file out again

				file_out << filedata << endl;
			}

			file_out.close();
		}
	}
	catch (const char* msg) {
		LOG(ERROR) << msg;

	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		std::string errormsg = "issues creating folders...\n";
		LOG(ERROR) << errormsg;

	}
	catch (...)
	{
		string errormessage = "issues creating folders!";
		LOG(ERROR) << errormessage;

	}

};

void UPackage::makeXacroURDF(Ptr<Design> design, Ptr<Application> app) {
	TiXmlDocument thisxacro;
	
	//xacro macro part
	{
		TiXmlDeclaration * decl2 = new TiXmlDeclaration("1.0", "", ""); //i don't think we can reuse anything...
		thisxacro.LinkEndChild(decl2);

		TiXmlElement * thisxacrorobot_root = new TiXmlElement("robot");
		thisxacrorobot_root->SetAttribute("name", "gummi");
		thisxacro.LinkEndChild(thisxacrorobot_root);

		TiXmlComment* comment = new TiXmlComment();
		comment->SetValue(" Some comment.");
		thisxacro.LinkEndChild(comment);

		TiXmlElement * thisxacromacro = new TiXmlElement("xacro:macro");
		thisxacromacro->SetAttribute("name", name.c_str());
		thisxacromacro->SetAttribute("params", "package_name");
		thisxacrorobot_root->LinkEndChild(thisxacromacro);

		//now parse the urdftree
		assert(elementsDict.size() > 0);
		for (auto el : elementsDict)
		{
			LOG(INFO) << "Parsing element:" + std::to_string(el.first);
			ULink* currLink = dynamic_cast<ULink*>(el.second);
			if (currLink)
			{
				LOG(INFO) << "calling genlink for link:" + currLink->name;
				bool succeeded = currLink->genlink(meshes_directory, components_directory, design, app);
				if (!succeeded)
				{
					string errormsg = "failed generating link" + currLink->name;
					LOG(ERROR) << errormsg;
					return;
				}
			}
			el.second->makexml(thisxacromacro, "${package_name}");
		};

		TiXmlElement * thisxacromacropar = new TiXmlElement(("xacro:" + name).c_str());
		thisxacromacropar->SetAttribute("package_name", name.c_str());
		thisxacrorobot_root->LinkEndChild(thisxacromacropar);

		string thissegmentxacroname = ("thissegment.urdf.xacro");
		string filenametosave = (xacro_directory / thissegmentxacroname).string();

		LOG(INFO) << "Saving file" + (filenametosave);
		thisxacro.SaveFile(filenametosave.c_str());
	}
	   
};

void UMainPackage::setpath(fs::path thisscriptpath_, fs::path  basemost_directory)
{

	LOG(DEBUG) << "called setpath";

	LOG(INFO) << "This script's path: " + (thisscriptpath_.string());

	thisScriptPath = thisscriptpath_;


	LOG(INFO) << "Basemost directory:" + basemost_directory.string();
	base_directory = basemost_directory / name;

	LOG(INFO) << "Base directory:" + base_directory.string();
	if (!fs::exists(base_directory))
		fs::create_directories(base_directory); //// will create whole tree if needed
	
	xacro_directory = base_directory / "xacro";
	config_directory = base_directory / "config";
	if (!fs::exists(xacro_directory))
		fs::create_directory(xacro_directory);
	if (!fs::exists(config_directory))
		fs::create_directory(config_directory);
}


void UMainPackage::makefiles(string _ms_packagename, std::vector<std::string> packagenamelist)
{
	try 
	{
		LOG(DEBUG) << "called makefiles";


		vector<string> filestochange = { "display.launch", "urdf_.rviz", "package.xml", "CMakeLists.txt" };

		for (auto myfilename : filestochange)
		{
			ifstream file_in;
			// Read in the file

			auto thisfilename = thisScriptPath / "resources" / "wholechain" / myfilename;

			LOG(INFO) << "Opening file:" + (thisfilename.string());
			file_in.open(thisfilename);
			if (!file_in)
				LOG(ERROR) << "failed to open input file:" + thisfilename.string();

			auto thisoutfilename = base_directory / myfilename;
			ofstream file_out(thisoutfilename.string(), std::ofstream::out);

			string filedata;

			while (std::getline(file_in, filedata))
			{
				// Replace the target string

				replaceAll(filedata, "wholechain_template", _ms_packagename);

				// Write the file out again

				file_out << filedata << endl;
			}

			if (myfilename == "CMakeLists.txt")
			{
				//todo: need to change package.xml and cmakelist to add dependencies (?), althought I think for now this is not necessary. we don't use anything that needs to be parsed 


				//vars:
				//%MAINPACKAGENAME%
				//%SUBPACKNAME%
				//%SUBPACKNAMELIST%

				//add the URDF custom target bit:
				std::string main_package_name = "";
				const char* urdfcustomtarget = R"<>(
################ TO GENERATE UDRF FILE #################
add_custom_target(%MAINPACKAGENAME%urdfgen ALL ${ ROSRUNDIR } / rosrun xacro xacro --inorder
${ CMAKE_CURRENT_SOURCE_DIR } / xacro / wholearm.urdf.xacro %SUBPACKNAMELIST% >
${ CMAKE_CURRENT_SOURCE_DIR } / config / wholearm.urdf
WORKING_DIRECTORY ${ PROJECT_SOURCE_DIR }
COMMENT ** Creating %MAINPACKAGENAME% URDF file.
VERBATIM
)
)<>";
				std::string uct = urdfcustomtarget;
				replaceAll(uct, "%MAINPACKAGENAME%", _ms_packagename);
				LOG(DEBUG) << "uct so far:\n" << uct;

				//now I need to construct all the lines for all the subpackages I have

				std::string subpackliststr = "";
				for (auto pack : packagenamelist)
				{
					std::string subpackline = "\n%SUBPACKNAME%_dir : = ${ CMAKE_CURRENT_SOURCE_DIR } / .. / %SUBPACKNAME%";
					replaceAll(subpackline, "%SUBPACKNAME%", pack);
					subpackliststr += subpackline;
				}
				replaceAll(uct, "%SUBPACKNAMELIST%", subpackliststr);
				LOG(DEBUG) << "uct so far:\n" << uct;
				//write our uct to CMakelists
				file_out << uct << endl;
			}
			file_out.close();
		}
	}
	catch (const char* msg) {
		LOG(ERROR) << msg;
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		std::string errormsg = "issues creating folders...\n";
		LOG(ERROR) << errormsg;
	}
	catch (...)
	{
		string errormessage = "issues creating folders!";
		LOG(ERROR) << errormessage;
	}
	return;
};

void UMainPackage::makeView(ULink* Base)
{
TiXmlDocument thisurdfdoc;

TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
thisurdfdoc.LinkEndChild(decl);

TiXmlElement * thisurdfdocrobot_root = new TiXmlElement("robot");
thisurdfdocrobot_root->SetAttribute("name", "gummi");
thisurdfdoc.LinkEndChild(thisurdfdocrobot_root);

TiXmlComment* comment = new TiXmlComment();
comment->SetValue(" Some comment.");
thisurdfdoc.LinkEndChild(comment);


ULink base_link;
base_link.name = "base_link";

base_link.makexml(thisurdfdocrobot_root, name);

UJoint setaxisjoint;
setaxisjoint.name = "set_worldaxis";
setaxisjoint.isset = true;
setaxisjoint.type = "fixed";
setaxisjoint.realorigin.rpy = std::to_string(PI / 2) + " 0 0";
setaxisjoint.parentlink = &base_link;
setaxisjoint.childlink = Base;
setaxisjoint.makexml(thisurdfdocrobot_root, name);

//now add all subpackage includes
std::vector<std::string> packagenamelist;
for (auto thisPackage : thisTree->packageTree)
{

	TiXmlElement * thisurdfxacromacro = new TiXmlElement("xacro:include");
	thisurdfxacromacro->SetAttribute("filename", ("$(arg " + thisPackage.name + "_dir)/xacro/thissegment.urdf.xacro").c_str());
	thisurdfdocrobot_root->LinkEndChild(thisurdfxacromacro);
	packagenamelist.push_back(thisPackage.name);
}

makefiles(name, packagenamelist);
//base_directory = (mypaths_zero / name)
//name is not properly set!

string thissegmentxacroname_view = ("whole_" + name + ".urdf.xacro");
string filenametosave_view = (xacro_directory / thissegmentxacroname_view).string();

LOG(INFO) << "Saving view file" + (filenametosave_view);
thisurdfdoc.SaveFile(filenametosave_view.c_str());

};

void UrdfTree::genMainPack() {
	mainPackage->name = "arm0_somename";
	mainPackage->makeView(dynamic_cast<ULink*>(getElementByName("base")));

};


UrdfTree::UrdfTree() {
	mainPackage = new UMainPackage();
	mainPackage->thisTree = this;
};