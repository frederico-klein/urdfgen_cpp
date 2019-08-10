#pragma once

#include <filesystem>
#include <iostream>
#include <fstream>

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>

#include "inc/easylogging/easylogging++.h"


using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

namespace fs = std::filesystem;


bool replace(std::string&, const std::string&, const std::string&);

void replaceAll(std::string& str, const std::string& from, const std::string& to);

std::string bigprint(std::string s);
std::string semibigprint(std::string s);

template <class myType> std::string asstring(myType v)
{
	return std::to_string(v);
};
//overloading instead of specializing, because I couldnot get the syntax to work //specialized template for strings.
std::string asstring(const std::string v);

template<typename T, typename A> std::string showarrayasstring(std::vector<T, A> v)
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
			if ((i + 1) % 4 == 0)
				myres += '\n';
			i++;
		}
		myres += "]";

	}
	return myres;
};

std::vector<std::string> splitstr(std::string, std::string);

std::string clearupst(std::string);
