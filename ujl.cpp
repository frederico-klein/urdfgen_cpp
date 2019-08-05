#include "ujl.h"
#include "inc/tinyxml.h"
#include "inc/easylogging/easylogging++.h"

//ulink stuff
std::string ULink::getitems()
{
	std::string iss = (coordinatesystem.isset) ? "Yes" : "No";
	std::string items = "isset:" + iss;
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


			visualXE->LinkEndChild(materialXE);
			
			//collision
			TiXmlElement* collisionXE = new TiXmlElement("collision");
			linkXE->LinkEndChild(collisionXE);

			TiXmlElement* voriginXE = new TiXmlElement("origin");
			voriginXE->SetAttribute("xyz", collision.origin.xyz.c_str());
			voriginXE->SetAttribute("rpy", collision.origin.rpy.c_str());

			collisionXE->LinkEndChild(voriginXE);

			TiXmlElement* vgeometryXE = new TiXmlElement("geometry");

			collisionXE->LinkEndChild(vgeometryXE);

			TiXmlElement* vgmeshXE = new TiXmlElement("mesh");
			vgmeshXE->SetAttribute("filename", collision.geometryfilename.c_str());

			vgeometryXE->LinkEndChild(vgmeshXE);
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

void ULink::genlink(std::string meshes_directory, std::string components_directory)
{
	LOG(ERROR) << "not implemented!";
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

		LOG(INFO) << "joint " + name + "successfully parsed as xml!";
	}
	catch (...)
	{
		//todo: 
		//LOG(ERROR) << "not implemented!";
		LOG(ERROR) << "joint " + name + "FAILED to be parsed as xml!";
	}
};