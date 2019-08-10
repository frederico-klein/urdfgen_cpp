#include "shared_funcs.h"

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

namespace fs = std::filesystem;

bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

void replaceAll(std::string& str, const std::string& from, const std::string& to) { //thnx stoverflow
	if (from.empty())
		return;
	size_t start_pos = 0;
	while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
		str.replace(start_pos, from.length(), to);
		start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
	}
}

std::string bigprint(std::string s)
{
	std::string lotsofequals = "=============================================================================";
	return "\n" + lotsofequals + "\n" + lotsofequals + "\n" + s + "\n" + lotsofequals + "\n" + lotsofequals + "\n";

};

std::string semibigprint(std::string s)
{
	std::string lotsofequals = "=============================================================================";
	return "\n" + lotsofequals + "\n" + s + "\n" + lotsofequals + "\n" ;

};

std::string asstring(const std::string v)
{
	return v;
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
	if (s != "")
		vstrings.push_back(s);
	return vstrings;
}

std::string clearupst(std::string s)
{
	std::vector<std::string> vstrings;
	std::string mystring = s;
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

	return mystring.substr(0, mystring.size() - 1);
}