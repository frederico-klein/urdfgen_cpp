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
namespace fs = std::filesystem;

typedef pair<int, UElement*> DicElement; 
typedef pair<vector<DicElement>, vector<DicElement>> TwoDic;
typedef vector<UJoint*> UJointList;
typedef vector<ULink*> ULinkList;

class UPackage;

class UrdfTree
{
public:
	//properties
	std::string report;
	vector<DicElement> elementsDict;
	vector<UPackage> packageTree;

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
	UElement* getElementByName(std::string);
	string getCurrentElDesc() ;
	void setCurrentEl(int) ;
	std::string UrdfTree::getdebugtext();
	UrdfTree() {};
	UrdfTree(UrdfTree&) = default;
	~UrdfTree() {};
	//functions for multipack
	vector<std::string> packageList();
	void genMainPack();

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

class UPackage {
public:
	vector<DicElement> elementsDict;
	std::string name;
	fs::path base_directory;
	fs::path meshes_directory;
	fs::path components_directory;
	fs::path xacro_directory;
	fs::path config_directory;
	ULink* fsFatherLink; //basically we want the name to create a virtual link with the same name, but to keep consistency, we probably should hold a pointer to the ULink object!

	//methods
	vector<UJoint*> alljoints();
	vector<ULink*> alllinks();
	virtual void makeView();
	virtual void makeXacroURDF(Ptr<Design> design, Ptr<Application> app);
	virtual void makeXacroSRDF();
	virtual void setpath(fs::path thisscriptpath, fs::path base_directory);
};

class UMainPackage :UPackage
{
	void makeView(ULink* Base);
	void setpath(string _ms_packagename, fs::path thisscriptpath, fs::path base_directory, std::vector<std::string> packagenamelist);
};