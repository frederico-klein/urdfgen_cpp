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

void ULink::makexml(TiXmlDocument* urdfroot)
{


	LOG(ERROR) << "not implemented!";
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
std::string UJoint::setjoint(Ptr<Joint> joint)
{

	generatingjointname = joint->name();
	//set origin 
	try
	{

		Ptr<JointGeometry> jointGeometry_var = joint->geometryOrOriginOne();
		//todo: set from origin 2 as well?
		Ptr<Point3D> thisorigin = jointGeometry_var->origin();
		origin.setxyz(thisorigin->x(), thisorigin->y(), thisorigin->z());
		
	}
	catch (...)
	{
		return "something wrong happened. you need to set this joint manually";
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
		Ptr<JointMotion> jointMotion_var = joint->jointMotion();
		//Ptr<Point3D> thisorigin = jointMotion_var->origin();
		//origin.setxyz(thisorigin->x, thisorigin->x, thisorigin->x);
		
		switch (jointMotion_var->jointType())
		{
			case 0: //
			{
				type = "fixed";
				break;
			}
			case 1: 
			{
				//type will be either "revolute" or continuous, depends on whether we have limits or not
				type = "continuous";
				bool haslimits = false;
				//tries to sets joint limits -> "revolute"
				Ptr<RevoluteJointMotion> thisjointmotion = joint->jointMotion();
				Ptr< JointLimits > thislimits;
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
				//case 2://
				//break;
			default:
			{
				return "joint type" +std::to_string(jointMotion_var->jointType()) + " not implemented!";
				break;
			}
		}
	}
	catch (...)
	{
		return "something wrong happened. you need to set this manually";
	}





	isset = true;
	return "";
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

void UJoint::makexml(TiXmlDocument* urdfroot)
{
	//todo: 
	LOG(ERROR) << "not implemented!";
};