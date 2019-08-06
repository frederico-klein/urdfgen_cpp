#include "ujl.h"
#include "inc/tinyxml.h"
#include "inc/easylogging/easylogging++.h"
#include <filesystem>

namespace fs = std::filesystem;

//ulink stuff
std::string ULink::getitems()
{
	std::string iss = (coordinatesystem.isset) ? "Yes" : "No";
	std::string items = "isset:" + iss + "\n";
	for (auto it = group.cbegin(); it != group.cend(); it++)
	{
		Ptr<Occurrence> thisocc = *it;
		items = items + thisocc->name() + "\n";
	}
	if (items == "")
		return "no occurrences assigned to link!\n";
	return items;
};

void ULink::genfatherjoint(UJoint joint)
{
	if (!joint.isset)
		LOG(ERROR) << "tried to set displacement for link:" + name + ",but joint " + joint.name + " is not set!";
	else
	{
		coordinatesystem = joint.origin;
	}
};

void ULink::makexml(TiXmlElement* urdfroot, std::string packagename)
{
	LOG(DEBUG) << "ULink makexml function was called!";

	try {
		//if I ever create different stls for collision and geometry, this will have to change:
		visual.geometryfilename = "package://" + packagename + "/meshes/" + name + ".stl";
		collision.geometryfilename = visual.geometryfilename;

		TiXmlElement * linkXE = new TiXmlElement("link");
		linkXE->SetAttribute("name", name.c_str());

		urdfroot->LinkEndChild(linkXE);

		if (!isVirtual)
		{
			TiXmlElement * inertialXE = new TiXmlElement("inertial");
			linkXE->LinkEndChild(inertialXE);

			TiXmlElement * originXE = new TiXmlElement("origin");
			originXE->SetAttribute("xyz", inertial.origin.xyz.c_str());
			originXE->SetAttribute("rpy", inertial.origin.rpy.c_str());

			inertialXE->LinkEndChild(originXE);

			TiXmlElement * massXE = new TiXmlElement("mass");
			massXE->SetAttribute("value", inertial.mass.c_str());

			inertialXE->LinkEndChild(massXE);

			TiXmlElement * inertiaXE = new TiXmlElement("inertia");
			inertiaXE->SetAttribute("ixx", inertial.inertia.ixx.c_str());
			inertiaXE->SetAttribute("ixy", inertial.inertia.ixx.c_str());
			inertiaXE->SetAttribute("ixz", inertial.inertia.ixx.c_str());
			inertiaXE->SetAttribute("iyy", inertial.inertia.ixx.c_str());
			inertiaXE->SetAttribute("iyz", inertial.inertia.ixx.c_str());
			inertiaXE->SetAttribute("izz", inertial.inertia.ixx.c_str());

			inertialXE->LinkEndChild(inertiaXE);

			//visual
			TiXmlElement* visualXE = new TiXmlElement("visual");
			linkXE->LinkEndChild(visualXE);

			TiXmlElement* voriginXE = new TiXmlElement("origin");
			voriginXE->SetAttribute("xyz", inertial.origin.xyz.c_str());
			voriginXE->SetAttribute("rpy", inertial.origin.rpy.c_str());

			visualXE->LinkEndChild(voriginXE);

			TiXmlElement* vgeometryXE = new TiXmlElement("geometry");
			
			visualXE->LinkEndChild(vgeometryXE);

			TiXmlElement* vgmeshXE = new TiXmlElement("mesh");
			vgmeshXE->SetAttribute("filename", visual.geometryfilename.c_str());

			vgeometryXE->LinkEndChild(vgmeshXE);

			TiXmlElement* materialXE = new TiXmlElement("material");
			materialXE->SetAttribute("name",visual.materialfilename.c_str());

			TiXmlElement* colorXE = new TiXmlElement("color");
			colorXE->SetAttribute("rgba",visual.color.c_str());

			materialXE->LinkEndChild(colorXE);

			visualXE->LinkEndChild(materialXE);
			
			//collision
			TiXmlElement* collisionXE = new TiXmlElement("collision");
			linkXE->LinkEndChild(collisionXE);

			TiXmlElement* coriginXE = new TiXmlElement("origin");
			coriginXE->SetAttribute("xyz", collision.origin.xyz.c_str());
			coriginXE->SetAttribute("rpy", collision.origin.rpy.c_str());

			collisionXE->LinkEndChild(coriginXE);

			TiXmlElement* cgeometryXE = new TiXmlElement("geometry");

			collisionXE->LinkEndChild(cgeometryXE);

			TiXmlElement* cgmeshXE = new TiXmlElement("mesh");
			cgmeshXE->SetAttribute("filename", collision.geometryfilename.c_str());

			cgeometryXE->LinkEndChild(cgmeshXE);
		}
		LOG(INFO) << "link " + name + "successfully parsed as xml!";
	}
	catch (...)
	{
		//todo: 
		//LOG(ERROR) << "not implemented!";
		LOG(ERROR) << "link " + name + "FAILED to be parsed as xml!";
	}
}

template <class myType> std::string asstring(myType v)
{
 return std::to_string(v);
};
//overloading instead of specializing, because I couldnot get the syntax to work //specialized template for strings.
std::string asstring(const std::string v)
{
	return v;
};

template <class T> std::string showarrayasstring(std::vector<T> v)
{
	std::string myres = "size:" + std::to_string(v.size()) + "\n";
	if (v.size() != 16)
	{
		myres += "{";
		for (auto num : v)
		{
			myres += asstring(num) + std::string(" ");
		}
		myres += "}";
	}
	else
	{
		myres += "[";
		int i = 0; //
		for (auto num : v)
		{
			myres += asstring(num) + std::string(" ");
			if ((i+1) % 4 == 0)
				myres += '\n';
			i++;
		}
		myres += "]";

	}
	return myres;
};

std::vector<std::string> splitstr(std::string s, std::string token)
{
	//thanks StackOverflow:
	std::vector<std::string> vstrings;
	size_t pos = 0;
	while ((pos = s.find(token)) != std::string::npos) {
		vstrings.push_back(s.substr(0, pos));
		s.erase(0, pos + token.length());
	}
	if (s!="")
		vstrings.push_back(s);
	return vstrings;
}

std::string clearupst(std::string s)
{
	std::vector<std::string> vstrings;
	std::string mystring =s;
	char* removethese = "[:!@#$.()/-]";
	char changethis = ' ', forthis = '_';

	for (char * removethis = removethese; *removethis != '\0'; removethis++)
	{
		vstrings = splitstr(mystring, std::string(1, *removethis));
		mystring = "";
		for (auto astring : vstrings)
		{
			mystring += astring;
		}
	}
	vstrings = splitstr(mystring, std::string(1, changethis));
	mystring = "";
	for (auto astring : vstrings)
	{
		mystring += astring + forthis;
	}

	return mystring.substr(0,mystring.size()-1);
}

void ULink::genlink(fs::path meshes_directory, fs::path components_directory, Ptr<Design> _design, Ptr<Application> _app)
{
	LOG(ERROR) << "not implemented!";

	isVirtual = false;
	try {
		LOG(DEBUG) << "starting genlink";
		// Get the root component of the active design;

		Ptr<Component> rootComp = _design->rootComponent();
		if (!rootComp)
			throw "error: can't find root component";

		Ptr<OccurrenceList> allOccs = rootComp->allOccurrences();
		if (!allOccs)
			throw "error: can't get all orccurrences";

		// create the exportManager for the original file that has the whole design;
		Ptr<ExportManager> exportMgr = _design->exportManager();
		if (!exportMgr)
			throw "error: can't find export manager";

		Ptr<Matrix3D> removejointtranslation = Matrix3D::create();
		Ptr<Vector3D> translation = Vector3D::create(-coordinatesystem.x, -coordinatesystem.y, -coordinatesystem.z);
		removejointtranslation->setToIdentity();
		removejointtranslation->translation(translation);
		LOG(DEBUG) << "Offset from joint tm is: " + showarrayasstring(removejointtranslation->asArray());
		LOG(DEBUG) << "Offset from joint translation is: " + showarrayasstring(removejointtranslation->translation()->asArray());

		////////// add occurrences from other stuff to this new stuff;

		//let's test the occurrence text splitter thingy:

		//std::string myfunnystring = "some+weird+string+with_a_lot+of+plusses!";
		//std::vector<std::string> a = splitstr(myfunnystring,"+");
		//LOG(INFO) << "my string split: " + showarrayasstring(a);

		//std::string myfunnystring2 = "some+weird";
		//std::vector<std::string> a2 = splitstr(myfunnystring2, "+");
		//LOG(INFO) << "my string split: " + showarrayasstring(a2);

		//std::string myfunnystring3 = "some++weird";
		//std::vector<std::string> a3 = splitstr(myfunnystring3, "+");
		//LOG(INFO) << "my string split: " + showarrayasstring(a3);

		std::vector<Ptr<Matrix3D>> it;
		for (int i = 0; i< group.size();i++)
		{
			auto occ = group[i];
			std::vector<std::string> pathsplit = splitstr(occ->fullPathName(),"+");

			std::vector<Ptr<Matrix3D>> newrotl;
			for (size_t j =1; j<pathsplit.size();j++) 
			{
				std::vector<std::string> thisoccnamelist = std::vector(pathsplit.begin(),pathsplit.begin()+j);
				std::string	thisoccname = thisoccnamelist[0];
				for (size_t k = 1; k < thisoccnamelist.size(); k++)	//		for ( k in range(1, len(thisoccnamelist)) )
					{
						thisoccname = thisoccname + "+" + thisoccnamelist[k];
					}
				LOG(INFO) << "\tTMS::: getting the tm for:" + thisoccname;
				for (size_t l = 1; l < allOccs->count(); l++) //for (l in range(0, allOccs.count)) {
					{   				
					if (allOccs->item(l)->fullPathName() == thisoccname)
						{
							//then i want to multiply their matrices!;
							Ptr<Matrix3D> lasttm = allOccs->item(l)->transform()->copy();
							newrotl.push_back(lasttm);
							LOG(DEBUG) << allOccs->item(l)->fullPathName();
							LOG(DEBUG) << "\twith tm:" + showarrayasstring(lasttm->asArray());
							LOG(DEBUG) << "\twith translation is:" + showarrayasstring(lasttm->translation()->asArray());
							//newrot.transformBy(allOccs.item(l).transform);
						}
					}
				////// now that i have all the occurrences names i need to get them from allOccs(?!);				
			}
			Ptr<Matrix3D> lasttransform = occ->transform()->copy();
			LOG(DEBUG) << "own tm (lasttransform) is:" + showarrayasstring(lasttransform->asArray());
			newrotl.push_back(lasttransform);
			//                newrot = removejointtranslation;
			Ptr<Matrix3D> newrot = Matrix3D::create();
			newrot->setToIdentity();

			// trying to uncomment the largest amount of lines the fastest without causing compile/runtime errors

			//checking indexing
			LOG(DEBUG) << "newrotl.size()=" + std::to_string(newrotl.size());
			//for (int j = newrotl.size(); j < 0; --j) 
			for (std::vector<Ptr<Matrix3D>>::reverse_iterator newrotlj = newrotl.rbegin();newrotlj != newrotl.rend(); ++newrotlj)  //for (int j = newrotl.size(); j < 0; j--)  //for (j in reversed(range(0, len(newrotl)))) // this was failing!
			{
				auto j = std::distance(newrotl.rbegin(), newrotlj); //with this newrotl[j] also will work, but I just wanted to try the new pointer syntax.
				LOG(DEBUG) << "\nj:" + std::to_string(j) + "newrot is:" + showarrayasstring(newrot->asArray()) + "newrotl[" + std::to_string(j) + "] is:" + showarrayasstring((*newrotlj)->asArray());
				newrot->transformBy(*newrotlj);
				//LOG(DEBUG) << "\nj:" + std::to_string(j) + "newrot is:" + showarrayasstring(newrot->asArray()) + "newrotl[" + std::to_string(j) + "] is:" + showarrayasstring(newrotl[j]->asArray());
				//newrot->transformBy(newrotl[j]);
			}
			newrot->transformBy(removejointtranslation);
			//		express = "it" + str(i) + "=newrot";
			//		exec(express);
			LOG(DEBUG) << "\nit transformation is:" + showarrayasstring(newrot->asArray());
			it.push_back(newrot);
		}


			//		//stlname = rootComp.name.translate(None, "{!@//$");
			//		//line = re.sub("[!@//$]", "", line);
		std::string	stlname = clearupst(name);
		LOG(INFO) << "stl name after removing weird characters:" + stlname;
		//LOG(INFO) << "my string clearupst'ed: " + clearupst("Funny:[name(2)-8].#Hi  Purpur");
			//		//fileName = components_directory+"/" + stlname;
		for (int i =0; i< group.size();i++ )
		{		
			std::filesystem::path fileName = components_directory / (stlname + std::to_string(i) + ".stp");
			LOG(INFO) << "saving file " + fileName.string();
			LOG(INFO) << "from occurrence" + group[i]->fullPathName();
			LOG(DEBUG) << "with tm:" + showarrayasstring(it[i]->asArray());
			Ptr<STEPExportOptions> stpOptions = exportMgr->createSTEPExportOptions(fileName.string(), group[i]->component());
			if (!stpOptions)
				throw "error: can't set step export options";

			bool isOk = exportMgr->execute(stpOptions);
			if (!isOk)
				throw "exportMgr failed to generate file" + fileName.string();
			LOG(INFO) << "File " + fileName.string() + " exported successfully";
		}

		//created all the components. now I need to open a new document. load them and export the whole rootcomponent as STL file.

		// Create a document.;

		Ptr<Documents> docs = _app->documents();
		if (!docs)
			throw "cant get documents";

		// Create a document.
		Ptr<Document> doc = docs->add(DocumentTypes::FusionDesignDocumentType);
		if (!doc)
			throw "cant add document";

		doc->name(stlname);
		Ptr<Design> design = _app->activeProduct();
		if (!design)
			throw "cant get current design";



		// Get the root component of the active design;
		Ptr<Component> rootComp2 = design->rootComponent();
			//		// Create two new components under root component;
			//		allOccs = rootComp.occurrences;
		// Get import manager;
		Ptr<ImportManager> importManager = _app->importManager();
		if (!importManager)
			throw "cant create importManager";
		//////// add occurrances from other stuff to this new stuff;
		for (int i = 0; i< group.size();i++) 
		{
			std::filesystem::path fileName = components_directory / (stlname + std::to_string(i) + ".stp");
			LOG(INFO) << "loading file: " + fileName.string();
			Ptr<STEPImportOptions> stpOptions = importManager->createSTEPImportOptions(fileName.string());
			if (!stpOptions)
				throw "error: can't set step import options";
			bool isOk = importManager->importToTarget(stpOptions, rootComp2);
			if (!isOk)
				throw "failed to import" + fileName.string();
		}
		assert(rootComp2->occurrences()->count()== group.size()); //it should be the same, right?
		for (int i = 0; i<rootComp2->occurrences()->count();i++)
		{
			//thistransf.transformBy(removejointtranslation)    ;
			rootComp2->occurrences()->item(i)->transform(it[i]);
		}
		double xx;
		double yy;
		double zz;
		double xy;
		double yz;
		double xz;
		double mass;
		rootComp2->physicalProperties()->getXYZMomentsOfInertia(xx, yy, zz, xy, yz, xz);
		mass = rootComp2->physicalProperties()->mass();
		LOG(INFO) << "XYZ moments of inertia: xx:" + std::to_string(xx) + "yy" + std::to_string(yy) + "zz" + std::to_string(zz) + "xy" + std::to_string(xy) + "yz" + std::to_string(yz) + "xz" + std::to_string(xz);
		LOG(INFO) << "Mass:" + std::to_string(mass);
		//setting this to inertial
		inertial.setall(mass, xx, xy, xz, yy, yz, zz);

		////// setting units to meters so stls will have proper sizes!;
		Ptr<FusionUnitsManager> unitsMgr = design->fusionUnitsManager();
		unitsMgr->distanceDisplayUnits(DistanceUnits::MeterDistanceUnits);
		// create aNOTHER! exportManager instance;
		Ptr<ExportManager> exportMgr2 = design->exportManager();
		if (!exportMgr2)
			throw "error: can't create second export manager";
		std::string meshname = (meshes_directory / stlname).string();
		Ptr<STLExportOptions> stlRootOptions = exportMgr->createSTLExportOptions(rootComp2, meshname);
			
		stlRootOptions->sendToPrintUtility(false);
		LOG(INFO) << "saving STL file: " + meshname;
		exportMgr->execute(stlRootOptions);
		//visual.geometryfilename = "package://" + _ms.packagename + "/meshes/" + stlname + ".stl";
		//collision.geometryfilename = visual.geometryfilename; // the legend has it that this file should be a slimmer version of the visuals, so that collisions can be calculated more easily....       ;
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		std::string errormsg = "issues running genstl\n";
		LOG(ERROR) << errormsg;
	}
	catch (char* msg)
	{
		LOG(ERROR) << msg;
	}
	catch (...) // is there an exception that is not derived from std::exception?
	{
		std::string errormsg = "issues running genstl";
		LOG(ERROR) << errormsg;
	}

};

void OrVec::setxyz(double xx, double yy, double zz)
{
	x = xx;
	y = yy;
	z = zz;
	xyz = std::to_string(x / 100) + " " + std::to_string(y / 100) + " " + std::to_string(z / 100);
	// the internal representation of joint occurrences offsets seems to be in cm no matter what you change the units to be. this needs to be checked, but i think it is always like this. if you are reading this line and wondering if this is the reason why your assembly looks like it exploded, then I was wrong...
	// there will be inconsistencies here and if you change the values below to be "right", then the translation part on .genlink will not work. be mindful when trying to fix it. 
	isset = true;
};
void OrVec::setrpy(double rr, double pp, double yy)
{
	r = rr;
	p = pp;
	yaw = yy;
	xyz = std::to_string(r / 180 * PI) + " " + std::to_string(p / 180 * PI) + " " + std::to_string(yaw / 180 * PI);
	// the internal representation of joint angles are in degrees, but the URDF is in radians...
	//isset = true;
};

//ujoint stuff
void UJoint::setjoint(Ptr<Joint> joint)
{
	LOG(DEBUG) << "started setjoint";
	generatingjointname = joint->name();
	entity = joint;
	//set origin 
	try
	{
		LOG(DEBUG) << "1";
		Ptr<JointGeometry> jointGeometry_var = joint->geometryOrOriginOne();
		//todo: set from origin 2 as well?
		LOG(DEBUG) << "2";

		Ptr<Point3D> thisorigin = jointGeometry_var->origin();
		LOG(DEBUG) << "3";

		origin.setxyz(thisorigin->x(), thisorigin->y(), thisorigin->z());
		LOG(DEBUG) << "4";

	}
	catch (...)
	{
		LOG(ERROR) << "something wrong happened while getting joint geometry. you need to set this joint manually. with sixdegree control. which is not implemented. so yeah, this is a failure";
	}

	//set joint type 
	//==============================================================================
	//         from the docs, we should implement this:
	//         Name     Value     Description
	//         BallJointType     6     Specifies a ball type of joint.
	//         CylindricalJointType     3     Specifies a cylindrical type of joint.
	//         PinSlotJointType     4     Specifies a pin - slot type of joint.
	//         PlanarJointType     5     Specifies a planar type of joint.
	//         RevoluteJointType     1     Specifies a revolute type of joint.
	//         RigidJointType     0     Specifies a rigid type of joint.
	//         SliderJointType     2     Specifies a slider type of joint.
	//==============================================================================

	try
	{
		LOG(DEBUG) << "5:trying to get jointmotion";

		Ptr<JointMotion> jointMotion_var = joint->jointMotion();
		//Ptr<Point3D> thisorigin = jointMotion_var->origin();
		//origin.setxyz(thisorigin->x, thisorigin->x, thisorigin->x);
		LOG(DEBUG) << "6: got jointmotion";

		switch (jointMotion_var->jointType())
		{
			case 0: //
			{
				LOG(DEBUG) << "0:7 joint is fixed";
				type = "fixed";
				break;
			}
			case 1: 
			{
				//type will be either "revolute" or continuous, depends on whether we have limits or not
				LOG(DEBUG) << "1:7 joint is either continuous or revolute depending on whether it has limits or not";

				type = "continuous";
				bool haslimits = false;
				//tries to sets joint limits -> "revolute"
				Ptr<RevoluteJointMotion> thisjointmotion = joint->jointMotion();
				LOG(DEBUG) << "1:8: revolutejointmotion casting worked.";
				Ptr<Vector3D> myaxis = thisjointmotion->rotationAxisVector();
				axis = std::to_string(myaxis->x()) + " " + std::to_string(myaxis->y()) + " " + std::to_string(myaxis->z());

				Ptr< JointLimits > thislimits = thisjointmotion->rotationLimits();
				LOG(DEBUG) << "1:9: jointlimits casting worked.";
				if (thislimits->isMinimumValueEnabled())
				{
					limit.lower = std::to_string(thislimits->minimumValue());
					haslimits = true;
				}
				if (thislimits->isMaximumValueEnabled())
				{
					limit.upper = std::to_string(thislimits->maximumValue());
					haslimits = true;
				}

				if (haslimits)
					type = "revolute";

				break;
			}
			default:
			{
				LOG(DEBUG) << "default:7";

				LOG(ERROR) << "joint type" +std::to_string(jointMotion_var->jointType()) + " not implemented!";
				break;
			}
		}
	}
	catch (...)
	{
		LOG(ERROR) << "something wrong happened. you need to set this manually";
	}
	isset = true;
	return;
};

//when sixdegree is working
void UJoint::setjoint(Ptr<Joint> joint, Ptr<CommandInput> cmdInput, Ptr<CommandInputs> inputs)
{
	LOG(ERROR) << "not implemented!";
	//not implemented
};

void UJoint::setrealorigin(OrVec fathercoordinatesystem)
{
	assert(fathercoordinatesystem.isset);
	realorigin.setxyz(origin.x-fathercoordinatesystem.x, origin.y - fathercoordinatesystem.y, origin.z - fathercoordinatesystem.z);
};

std::string UJoint::getitems()
{
	std::string iss = (isset) ? "Yes" : "No";
	std::string items = "isset:" + iss + "\tgenjn:" + generatingjointname + "\n" + "parent:" + parentlink + "\t" + "child:" + childlink + "\n";
	return items;
};

void UJoint::makexml(TiXmlElement* urdfroot, std::string ) //don't need packagename. it just so that both makexml functions have the same call. also, not sure if it will be needed
{
	LOG(DEBUG) << "UJoint makexml function was called!";
	try {
		TiXmlElement * jointXE = new TiXmlElement("joint");
		jointXE->SetAttribute("name", name.c_str());
		jointXE->SetAttribute("type", type.c_str());

		urdfroot->LinkEndChild(jointXE);

		TiXmlElement * originXE = new TiXmlElement("origin");
		originXE->SetAttribute("xyz", realorigin.xyz.c_str());
		originXE->SetAttribute("rpy", realorigin.rpy.c_str());

		jointXE->LinkEndChild(originXE);

		TiXmlElement * parentXE = new TiXmlElement("parent");
		parentXE->SetAttribute("link", parentlink.c_str());

		jointXE->LinkEndChild(parentXE);

		TiXmlElement * childXE = new TiXmlElement("child");
		childXE->SetAttribute("link", childlink.c_str());

		jointXE->LinkEndChild(childXE);

		TiXmlElement * limitXE = new TiXmlElement("limit");
		limitXE->SetAttribute("lower", limit.lower.c_str());
		limitXE->SetAttribute("upper", limit.upper.c_str());
		limitXE->SetAttribute("effort", limit.effort.c_str());
		limitXE->SetAttribute("velocity", limit.velocity.c_str());

		jointXE->LinkEndChild(limitXE);

		TiXmlElement * axisXE = new TiXmlElement("axis");
		axisXE->SetAttribute("xyz", axis.c_str());

		jointXE->LinkEndChild(axisXE);

		LOG(INFO) << "joint " + name + "successfully parsed as xml!";
	}
	catch (...)
	{
		//todo: 
		//LOG(ERROR) << "not implemented!";
		LOG(ERROR) << "joint " + name + "FAILED to be parsed as xml!";
	}
};

void Inertia::set(double xx_, double xy_, double xz_, double yy_, double yz_, double zz_) 
{
	ixx = std::to_string(xx_);
	ixy = std::to_string(xy_);
	ixz = std::to_string(xz_);
	iyy = std::to_string(yy_);
	ixx = std::to_string(yz_);
	ixx = std::to_string(zz_);
	LOG(INFO) << "inertia set for all xyz components";
};

void Inertial::setall(double mass_, double xx_, double xy_, double xz_, double yy_, double yz_, double zz_)
{
	mass = std::to_string(mass_);
	inertia.set(xx_,xy_, xz_, yy_, yz_, zz_);
	LOG(INFO) << "inertial set with mass and inertia ";
};