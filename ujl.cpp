#include "ujl.h"

std::string ULink::getitems()
{
	std::string items = "";
	for (auto it = group.cbegin(); it != group.cend(); it++)
	{
		items = items + it->name() + "\n";
	}
	return items;
};

void ULink::genfatherjoint(UJoint joint)
{
	//this is not it...
	//coordinatesystem = joint.origin;
};

void UElement::setElement(std::string eltype)
{
	if (eltype == "joint")
	{
		UJoint el;
		type = DT_JOINT;
	}
	else if (eltype == "link")
	{
		ULink el;
		type = DT_LINK;
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
}
void OrVec::setrpy(double rr, double pp, double yy)
{
	r = rr;
	p = pp;
	yaw = yy;
	xyz = std::to_string(r / 180 * PI) + " " + std::to_string(p / 180 * PI) + " " + std::to_string(yaw / 180 * PI);
	// the internal representation of joint angles are in degrees, but the URDF is in radians...
	//isset = true;
}