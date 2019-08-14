#pragma once
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "ujl.h"
#include "inc/easylogging/easylogging++.h"

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;
using namespace std;

typedef pair<int, UElement*> DicElement; 
typedef pair<vector<DicElement>, vector<DicElement>> TwoDic;
typedef vector<UJoint*> UJointList;
typedef vector<ULink*> ULinkList;

class UrdfTree
{
public:
	//properties
	std::string report;
	vector<DicElement> elementsDict;
	UElement* currentEl;
	Ptr<UserInterface> ui; 
	// methods
	void removeElementByName(vector<DicElement>*, string);
	void addLink(string name, int row);
	void addJoint(string name, int row);
	void rmElement(int elnum);
	std::string genTree();
	pair<string, ULinkList> allLinks() ;
	vector<string> allLinksvec();
	pair<string, UJointList> allJoints() ;
	pair<string, vector<UElement*>> allElements() ;
	UElement* getEl(int) ;
	string getCurrentElDesc() ;
	void setCurrentEl(int) ;
	std::string UrdfTree::getdebugtext();
	UrdfTree() {};
	UrdfTree(UrdfTree&) = default;
	~UrdfTree() {};
private:
	TwoDic gentreefindbase(vector<DicElement>);
	TwoDic gentreecore(pair<UJointList, TwoDic>) ;
	TwoDic gentreecorecore(TwoDic, UJoint*, bool*) ;
	void genfatherjoint(string, UJoint*) ;
	DicElement findjointscore(vector<DicElement>*, vector<DicElement>*) ;
	pair<UJointList,TwoDic> findjoints(TwoDic) ;
	pair<pair<string, ULinkList>, vector<string>> alllinks(vector<DicElement>) ;
	pair<pair<string, UJointList>, vector<string>> alljoints(vector<DicElement>);
	pair<pair<string, vector<UElement*>>, vector<string>> UrdfTree::allelements(vector<DicElement>);
	string UrdfTree::alljoints(UJointList);
};