#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "urdftree.h"
#include "ujl.h"

#include <filesystem>

#include <iostream>
#include <fstream>
#include "shared_funcs.h"

INITIALIZE_EASYLOGGINGPP;

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;
namespace fs = std::filesystem;

Ptr<Application> app;
Ptr<UserInterface> ui;
Ptr<Design> design;

// this file has mostly gui things.

const bool runfrommenu = false; // this allowed to be run as script as well. TODO: KEEP? 

class MotherShip
{
public:
	int rowNumber = 0, elnum = 0, oldrow = -1, numlinks = -1, numjoints = -1,lastrow = 0;
	std::string packagename = "mypackage";
	fs::path thisscriptpath;
	//missing! jtctrl lastjoint is maybe not a ujoint object?
	UJoint lastjoint;
	UrdfTree thistree;
	MotherShip() {
		rowNumber = 0, elnum = 0, oldrow = -1, numlinks = -1, numjoints = -1, lastrow = 0;	

		//setting thisscriptpath
		
		fs::path appdatadir = "";// getenv("USER");
		char* buf = nullptr;
		size_t sz = 0;

		if (_dupenv_s(&buf, &sz, "APPDATA") == 0 && buf != nullptr)
		{
			//ui->messageBox("EnvVarName = " + std::string(buf));
			appdatadir = buf;
			free(buf);
		}
		//LOG(INFO) << "appdatadir:"+ appdatadir.string();
		thisscriptpath = appdatadir / "Autodesk" / "Autodesk Fusion 360" / "API" / "AddIns" / "urdfgen_cpp"; //////// TODO: change xcopy command to have the resources also available in the webdeploy folder!!!
		//LOG(INFO) << "This script's path: "+ (thisscriptpath.string());

	};
	~MotherShip() {};
	void addRowToTable(Ptr<TableCommandInput>, std::string);
} _ms;

class SixDegree : OrVec
{
	/*
	This is a 6 degree of freedom control. I've seen this in fusion and it looks nicer, but I am not sure it made it into the API. 
	Reiventing the wheel here; kind of in a time crunch, just porting this code, 
	TODO: use the real control from Fusion. 
	
	*/
public:
	std::string name;
	void setxyzrpy()
	{

	};
	void interact()
	{

	};
	void setdist(std::string var)
	{
		Ptr<DistanceValueCommandInput> distanceValueInput;
	};
	void setangle(std::string var)
	{
		Ptr<AngleValueCommandInput> angleValueInput;
	};

	SixDegree(Ptr<CommandInputs> ownctrl_, std::string name_)
	{
		name = name_;
		ownctrl = ownctrl_;
		//maybe it has a name. not sure I might need to use this to create offsets for links later. If everything works correctly this won't be necessary though.

		// creates distance controls for X, Y, Z offsets

		addDistanceControl("X", 1, 0, 0);
		addDistanceControl("Y", 0, 1, 0);
		addDistanceControl("Z", 0, 0, 1);

		//creates angle controls for Roll, Pitch and Yaw

		addAngleControl("Roll"	, 0, 1, 0, 0, 0, 1);
		addAngleControl("Pitch"	, 0, 0, 1, 1, 0, 0);
		addAngleControl("Yaw"	, 1, 0, 0, 0, 1, 0);
	};

private:
	bool allvisible = true;
	bool allenabled = false;
	Ptr<CommandInputs> ownctrl;
	std::vector<Ptr<DistanceValueCommandInput>> distVect;
	std::vector<Ptr<AngleValueCommandInput>> angVect;

	void addDistanceControl(std::string var, double x, double y, double z)
	{
		Ptr<DistanceValueCommandInput> distanceValueInput = ownctrl->addDistanceValueCommandInput("distanceValue"+var, var, ValueInput::createByReal(0));
		distanceValueInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(x, y, z));
		distanceValueInput->hasMinimumValue(false);
		distanceValueInput->hasMaximumValue(false);
		distanceValueInput->isVisible(allvisible);
		distanceValueInput->isEnabled(allenabled);
		distVect.push_back(distanceValueInput); //not sure if I will need this, but just in case
	}
	void addAngleControl(std::string name, double x1, double y1, double z1, double x2, double y2, double z2)
	{
		Ptr<AngleValueCommandInput> angleValueInput = ownctrl->addAngleValueCommandInput("angleValue" + name, name, ValueInput::createByReal(0));
		angleValueInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(x1, y1, z1), Vector3D::create(x2, y2, z2));
		angleValueInput->hasMinimumValue(false);
		angleValueInput->hasMaximumValue(false);
		angleValueInput->isVisible(allvisible);
		angleValueInput->isEnabled(allenabled);
		angVect.push_back(angleValueInput); //not sure if I will need this, but just in case
	}

};

void MotherShip::addRowToTable(Ptr<TableCommandInput> tableInput, std::string LinkOrJoint)
{
	//Adds element to tree, i.e., row to table
	bool islink;
	std::string elname;
	if (!tableInput)
		return;
	// Get the CommandInputs object associated with the parent command.
	Ptr<CommandInputs> cmdInputs = tableInput->commandInputs();

	// Setting up controls to be added for each row
	Ptr<CommandInput> elnnumInput = cmdInputs->addStringValueInput("elnum" + std::to_string(elnum), "elnumTable" + std::to_string(elnum), std::to_string(elnum));
	elnnumInput->isEnabled(false);

	if (LinkOrJoint == "" || LinkOrJoint == "Link")
	{
		islink = true;
		numlinks += 1;
		if (elnum == 0 || thistree.allLinks().second.size() ==0 )// thistree.elementsDict.size()==0) // should check for number of links actually
			elname = "base"; //first link has to be named base
		else
			elname = "link" + std::to_string(numlinks);
	}
	else if (LinkOrJoint == "Joint")
	{
		islink = false;
		numjoints += 1;
		elname = "joint" + std::to_string(numjoints);
	}
	Ptr<DropDownCommandInput> JorLInput = cmdInputs->addDropDownCommandInput("TableInput_value" + std::to_string(elnum), "JorLTable" + std::to_string(elnum), DropDownStyles::TextListDropDownStyle);
	Ptr<ListItems> dropdownItems = JorLInput->listItems();
	if (!dropdownItems)
		return;
	dropdownItems->add("Link", islink, "");
	dropdownItems->add("Joint", !islink, "");
	JorLInput->isEnabled(false);

	Ptr<CommandInput> stringInput = cmdInputs->addStringValueInput("TableInput_string" + std::to_string(elnum), "StringTable" + std::to_string(elnum), elname);
	stringInput->isEnabled(false); //I'm disabling the ability to change element's name randomly...

	Ptr<CommandInput> slbutInput = cmdInputs->addBoolValueInput("butselectClick" + std::to_string(elnum), "Select", false, "", true);

    // Add the inputs to the table.
	lastrow = tableInput->rowCount();
	tableInput->addCommandInput(elnnumInput, lastrow, 0);
	tableInput->addCommandInput(JorLInput, lastrow, 1);
	tableInput->addCommandInput(stringInput, lastrow, 2);
	tableInput->addCommandInput(slbutInput, lastrow, 3);

	// Increment a counter used to make each row unique.

	rowNumber = rowNumber + 1;
	elnum += 1;

};


vector<fs::path> createpaths(string _ms_packagename, fs::path thisscriptpath)
{
	vector<fs::path> returnvectstr;

	try {
		LOG(DEBUG) << "called createpaths";
		fs::path userdir = "";// getenv("USER");
		//fs::path appdatadir = "";// getenv("USER");

		char* buf = nullptr;
		size_t sz = 0;
		if (_dupenv_s(&buf, &sz, "USERPROFILE") == 0 && buf != nullptr)
		{
			//ui->messageBox("EnvVarName = " + std::string(buf));
			userdir = buf;
			free(buf);
		}
		//if (_dupenv_s(&buf, &sz, "APPDATA") == 0 && buf != nullptr)
		//{
		//	//ui->messageBox("EnvVarName = " + std::string(buf));
		//	appdatadir = buf;
		//	free(buf);
		//}
		//string autodesk_dir = "Autodesk\\Autodesk Fusion 360\\API\\AddIns\\urdfgen_cpp"; ///very bad...

		auto folderDlg = ui->createFolderDialog();
		folderDlg->title("Choose location to save your URDF new package");
		folderDlg->initialDirectory(userdir.string());
		DialogResults dlgResult = folderDlg->showDialog();
		if (dlgResult == DialogError)
			LOG(ERROR) << "failed to create dialog";
		if (dlgResult != DialogOK) //this 0=0 check is failing for some reason!!
		{
			//ui->messageBox("dialogOK is resolved to "+std::to_string(adsk::core::DialogResults::DialogOK));
			ui->messageBox("you need to select a folder!");
			throw std::runtime_error("Directory not selected. cannot continue.\nError code: DialogResults enum" +std::to_string(dlgResult));
		}

		fs::path outputdir = folderDlg->folder();
		fs::path base_directory = outputdir / _ms_packagename;
		//LOG(INFO) << "appdatadir:"+ appdatadir.string();
		//ui->messageBox(appdatadir.string());
		//fs::path thisscriptpath = appdatadir / "Autodesk" / "Autodesk Fusion 360" / "API" / "AddIns" / "urdfgen_cpp"; //////// TODO: change xcopy command to have the resources also available in the webdeploy folder!!!
		LOG(INFO) << "This script's path: "+ (thisscriptpath.string());
		//ui->messageBox(thisscriptpath.string());
															 //fs::path thisscriptpath = fs::current_path();

		//ui->messageBox(base_directory.string());
		LOG(INFO) <<"Base directory:"+base_directory.string();
		if (!fs::exists(base_directory))
			fs::create_directories(base_directory); //// will create whole tree if needed
		fs::path meshes_directory = base_directory / "meshes";
		//ui->messageBox(meshes_directory.string());
		fs::path components_directory = base_directory / "components";
		//ui->messageBox(components_directory.string());
		if (!fs::exists(meshes_directory))
			fs::create_directory(meshes_directory);
		if (!fs::exists(components_directory))
			fs::create_directory(components_directory);
		vector<string> filestochange = { "display.launch", "urdf_.rviz", "package.xml", "CMakeLists.txt" }; //actually urdf.rviz is the same, but i didnt want to make another method just to copy.when i have more files i need to copy i will do it.
		//myfilename = "display.launch";
		for (auto myfilename : filestochange)
		{
			ifstream file_in;
			// Read in the file

			auto thisfilename = thisscriptpath / "resources" / myfilename;
			//ui->messageBox(thisfilename.string());
			LOG(INFO)<<"Opening file:"+(thisfilename.string());
			file_in.open(thisfilename);
			if (!file_in)
				LOG(ERROR) << "failed to open input file:" + thisfilename.string();

			auto thisoutfilename = base_directory / myfilename;
			ofstream file_out(thisoutfilename.string(), std::ofstream::out);
			
			string filedata;
			
			while (std::getline(file_in,filedata))
			{
				//if i use getline i don't need this anymore...
				//file_in >> filedata;

				// Replace the target string

				replaceAll(filedata, "somepackage", _ms_packagename);

				// Write the file out again

				file_out << filedata << endl;
			}
			file_out.close();
		}
		returnvectstr = { base_directory, meshes_directory, components_directory };
	}
	catch (const char* msg) {
		LOG(ERROR) << msg;
		ui->messageBox(msg);
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		std::string errormsg = "issues creating folders...\n";
		LOG(ERROR) << errormsg;
		ui->messageBox(errormsg);
	}
	catch (...)
	{
		string errormessage = "issues creating folders!";
		LOG(ERROR) << errormessage;
		ui->messageBox(errormessage);
	}
return returnvectstr;
};

// InputChange event handler.
class UrdfGenOnInputChangedEventHander : public adsk::core::InputChangedEventHandler
{
public:
	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override
	{
		bool rows_differ = false;
		//not efficient check, but
		if (_ms.oldrow !=_ms.rowNumber)
			rows_differ = true;

		std::string jointname;
		Ptr<CommandInputs> inputs = eventArgs->inputs();
		if (!inputs)
			return;

		Ptr<CommandInput> cmdInput = eventArgs->input();
		if (!cmdInput)
			return;

		//ui->messageBox("old one"+cmdInput->id());

		Ptr<TableCommandInput> tableInput = inputs->itemById("table");
		//if (!tableInput)
		//	return;
		LOG_IF(!tableInput, DEBUG) << "no table input. inside a group, probably";

		//ui->messageBox("1");
		Ptr<TextBoxCommandInput> debugInput = inputs->itemById("debugbox");
		//ui->messageBox("2");



		//define the groups first:
		Ptr<GroupCommandInput> linkgroupInput = inputs->itemById("linkgroup");
		Ptr<GroupCommandInput> jointgroupInput = inputs->itemById("jointgroup");
		//ui->messageBox("3");

		//ui->messageBox("4");
		Ptr<SelectionCommandInput> linkselInput;
		Ptr<SelectionCommandInput> jointselInput;
		Ptr<DropDownCommandInput> cln;
		Ptr<DropDownCommandInput> pln;
		//ui->messageBox("5");
		//inside the group, there is no group!
		if (!linkgroupInput) // linkgroupInput is None:		
		{
			LOG(DEBUG) << "probably inside linkgroup!";
			linkselInput = inputs->itemById("linkselection");
		}
		else
			linkselInput = linkgroupInput->children()->itemById("linkselection");
		LOG_IF(!linkselInput, WARNING) << "linkselInput is a null pointer, either things are gonna fail, or I am inside jointgroup";

		//ui->messageBox("6:: so far so good");
		LOG(DEBUG) << "6:: so far so good";
		//inside the group, there is no group!
		if (!jointgroupInput) // linkgroupInput is None:			
		{
			//ui->messageBox("6.1:: so far so good");
			if (!linkselInput) 
			{
				//ui->messageBox("6.2:: so far so good");
				LOG(DEBUG) << "probably inside jointgroup!";
				jointselInput = inputs->itemById("jointselection");
				cln = inputs->itemById("childlinkname");
				if (!cln)
				{
					ui->messageBox("cln is nope");
					LOG(WARNING) << "cln is nope";
				}
				pln = inputs->itemById("parentlinkname");
				if (!pln)
				{
					ui->messageBox("pln is nope");
					LOG(WARNING) << "pln is nope";
				}
			}
		}
		else
		{
			jointselInput = jointgroupInput->children()->itemById("jointselection");
			cln = jointgroupInput->children()->itemById("childlinkname");
			pln = jointgroupInput->children()->itemById("parentlinkname");
		}
		//ui->messageBox("7::not here");
		LOG_IF(!jointselInput, ERROR) << "jointselInput is a null pointer, things are gonna fail";

		LOG(DEBUG) << "this command does not crash the beginning:" + cmdInput->id();

		if (tableInput)
		{
			int currrow = tableInput->selectedRow();
			//ui->messageBox("0");

			Ptr<StringValueCommandInput> thisstringinput = tableInput->getInputAtPosition(currrow, 0);
			//ui->messageBox("1");
			if (thisstringinput)
				int elementtobedefined = std::stoi(thisstringinput->value());
		}
		//ui->messageBox("2");


		// Reactions
		if (cmdInput->id() == "tableLinkAdd") {
			try {
				_ms.addRowToTable(tableInput, "Link");
				//ui->messageBox("1");
				{
					//actually I changed addrow and removed the add element button, so I don't really need to do this.
					Ptr<DropDownCommandInput> JorLInput = tableInput->getInputAtPosition(_ms.lastrow, 1);
					if (!JorLInput) {
						std::string thiserror = "error getting dropdown command box in position" + std::to_string(_ms.lastrow);
						throw thiserror;
					}
						
					JorLInput->isEnabled(false); 
				}
				//ui->messageBox("2");
				Ptr<StringValueCommandInput> thisstringinput = tableInput->getInputAtPosition(_ms.lastrow, 2);
				if (!thisstringinput)
					throw "error getting row!";
				//ui->messageBox("3");
				jointname = thisstringinput->value();
				//ui->messageBox("4");
				//ui -> messageBox(jointname);
				//otherfunc(jointname);
				//_ms.thistree.addJoint("but this?", _ms.elnum - 1);
				_ms.thistree.addLink(jointname, _ms.elnum - 1);
				//ui->messageBox("5");
			}
			catch (const char* msg) {
				LOG(ERROR) << msg;
				ui->messageBox(msg);
			}
			catch (...)
			{
				string errormessage = "issues adding link!";
				LOG(ERROR) << errormessage;
				ui->messageBox(errormessage);
			}
		}
		else if (cmdInput->id() == "tableJointAdd") {
			try {
				_ms.addRowToTable(tableInput, "Joint");
				tableInput->getInputAtPosition(_ms.lastrow, 1)->isEnabled(false);
				Ptr<StringValueCommandInput> thisstringinput = tableInput->getInputAtPosition(_ms.lastrow, 2);
				if (!thisstringinput)
					throw "error getting row!";
				jointname = thisstringinput->value();
				//ui -> messageBox(jointname);
				//otherfunc(jointname);
				//_ms.thistree.addJoint("but this?", _ms.elnum - 1);
				LOG(DEBUG) << "added joint:" + jointname;
				_ms.thistree.addJoint(jointname, _ms.elnum - 1);
			}
			catch (const char* msg) {
				LOG(ERROR) << msg;
				ui->messageBox(msg);
			}

			catch (...)
			{
				string errormessage = "issues adding joint!";
				LOG(ERROR) << errormessage;
				ui->messageBox(errormessage);
			}
		}
		else if (cmdInput->id() == "tableDelete") {
			if (tableInput->selectedRow() == -1) {
				ui->messageBox("Select one row to delete.");
			}
			else {
				int elementtobedeleted = tableInput->selectedRow();
				//first I set the current link to the one above it
				Ptr<StringValueCommandInput> thisstringinput;
				if (tableInput->selectedRow() != 0)
					thisstringinput = tableInput->getInputAtPosition(tableInput->selectedRow()-1, 0);
				else
				{
					//if it is the first row, we can't get the one above it
					thisstringinput = tableInput->getInputAtPosition( 1, 0);
				}
				if (thisstringinput)
				{
					std::string oneaboveorbelownumstr = thisstringinput->value();
					_ms.thistree.setCurrentEl(std::stoi(oneaboveorbelownumstr));
				}
				else
				{
					//table is empty
					_ms.thistree.setCurrentEl(-1);
				}

				//now I can delete the row
				tableInput->deleteRow(elementtobedeleted);
				//and delete the element
				_ms.thistree.rmElement(elementtobedeleted);
				_ms.rowNumber -= 1;
				
				//I also want to update the debug message text!
				debugInput->text(_ms.thistree.getdebugtext());
				LOG(DEBUG) << "deleted element okay";
			}
		}
		//ui->messageBox("is this being reached?");
		if (cmdInput->id() == "linkselection") 
		{

			ULink* currLink = dynamic_cast<ULink*>(_ms.thistree.currentEl);
			if (currLink)
			{
				currLink->group.clear();
				for (int i = 0; i < linkselInput->selectionCount();i++) 
				{
					Ptr<Occurrence> thisFusionOcc = linkselInput->selection(i)->entity();
					LOG(DEBUG) << "adding link entity:" + thisFusionOcc->name();
					currLink->group.push_back(linkselInput->selection(i)->entity());
				}
			}
			LOG(DEBUG) << "linkselection: done ";
			//ui->messageBox("not implemented yet");
		}
		else if (cmdInput->id() == "jointselection" && jointselInput->selectionCount() == 1)
		{
			//ui->messageBox("started");
			Ptr<Joint> thisFusionJoint = jointselInput->selection(0)->entity();
			LOG(DEBUG) << "adding joint entity:"+ thisFusionJoint->name();
			//ui->messageBox("is it the cast?");
			UJoint* thisjoint = dynamic_cast<UJoint*>(_ms.thistree.currentEl);
			//ui->messageBox("is it something else?");
			if (thisjoint)
			{
				thisjoint->setjoint(thisFusionJoint);
				//thisjoint->setjoint(jointselInput->selection(0)->entity, cmdInput.id, inputs)
			}
			else
			{
				string errormsg = "the cast didn't work. it should have.";
				LOG(ERROR) << errormsg;
				ui->messageBox(errormsg);
			}
			LOG(DEBUG) << "jointselection: done ";
			//ui->messageBox("done");
		}
		else if (cmdInput->id() == "parentlinkname") 
		{
			LOG(DEBUG) << "i am aware i am a parentlinkname control";
			//since I changed the visibility of controls I know current element is a joint, if this is accessible
			UJoint* thisjoint = dynamic_cast<UJoint*>(_ms.thistree.currentEl);
			if (thisjoint)
			{
				Ptr<ListItem> dropdown_parent1 = pln->selectedItem();
				std::string myparentlinkname = dropdown_parent1->name();
				thisjoint->parentlink = myparentlinkname;
				_ms.thistree.currentEl = thisjoint;
			}
			else
			{
				string errormsg = "the cast didn't work";
				LOG(ERROR) << errormsg;
				ui->messageBox(errormsg);
			}
			LOG(DEBUG) << "parentlinkname: done ";
		}
		else if (cmdInput->id() == "childlinkname") 
		{
			LOG(DEBUG) << "i am aware i am a childlinkname control";

			//since I changed the visibility of controls I know current element is a joint, if this is accessible
			UJoint* thisjoint = dynamic_cast<UJoint*>(_ms.thistree.currentEl);
			if (thisjoint)
			{
				Ptr<ListItem> dropdown_child2 = cln->selectedItem();
				std::string childlinkname = dropdown_child2->name();
				thisjoint->childlink = childlinkname;
				LOG(DEBUG) << "thisjointchildlink:"+ thisjoint->childlink +"\ndropdownthing:"+ dropdown_child2->name();
				_ms.thistree.currentEl = thisjoint;
			}
			else
			{
				string errormsg = "the cast didn't work";
				LOG(ERROR) << errormsg;
				ui->messageBox(errormsg);
			}
			LOG(DEBUG) << "childlinkname: done ";
		}
		else if (cmdInput->id() == "createtree") 
		{
			linkgroupInput->isVisible(false);
			jointgroupInput->isVisible(false);
			ui->messageBox(_ms.thistree.genTree());
		}		
		else if (cmdInput->id().find("butselectClick") != std::string::npos) ///// TODO: add all other controls from table here, if we ever want to make them selectable/changeable then how it is, will break!
		{
		//one liner with a nameless object. I am trying to see if that string is in the name of the command (because I append numbers to those controls, so that they are unique; also, not my idea, came with the fusion example code)
			//ui->messageBox("worked!");
			//I want to update the debug message text here. 
			//first we set current element
			if (tableInput)
			{
				std::string num_str = cmdInput->id().substr(14); // I used to get the value of the index from the control of the selected row. I guess I can double check
				if (tableInput->selectedRow() != -1)
				{
					Ptr<StringValueCommandInput> thisstringinput = tableInput->getInputAtPosition(tableInput->selectedRow(), 0);
					if (thisstringinput)
					{
						std::string num_str2 = thisstringinput->value();
						//assert(num_str2 == num_str); 
						if (num_str2 != num_str)
						{
							rows_differ = true;
							//actually, getting the number from the controlname string is better, since the value of the selected row will only change after this input events are processed!
							//ui->messageBox("numstr:" + num_str + "\nnumstr2:" + num_str2);
							LOG(DEBUG) << "numstr:" + num_str + "\nnumstr2:" + num_str2;
							_ms.oldrow = std::stoi(num_str);
							_ms.rowNumber = std::stoi(num_str2);
						}
					}
				}
				_ms.thistree.setCurrentEl(std::stoi(num_str));
				LOG(DEBUG) << "current selected element's name is:"+_ms.thistree.currentEl->name;
				debugInput->text(_ms.thistree.getdebugtext());
			}
		}
		//else if (cmdInput->id() == "butselectClick") {}
		else if (cmdInput->id() == "packagename")
		{
			//true I only need to instantiate the variables before if they are needed in more than one place...
			Ptr<StringValueCommandInput> thisstringinput = inputs->itemById("packagename");
			_ms.packagename = thisstringinput->value();
		}

		// SO I am missing all of the things for the joint control...

		//else if (cmdInput->id() == "") {}

		//checks current element and updates all things!

		LOG(DEBUG) << "entering the old setcurrelement from Mothership that, because of the multiple pointers needed to be passed came back here";
		//this is not the nicest piece of code, but it will have to do. 
		bool runonce = true;
		while (runonce) {
			runonce = false;
			try {
				if (_ms.thistree.currentEl)
				{
					if (rows_differ)
					{
						if (!linkselInput || !jointselInput)
						{
							//no need to throw an error here, this happens in harmless situations. i'll just break out
							break;
							//throw "either linkselInput or jointselInput are null. this will fail, aborting";
						}
						if (!linkgroupInput || !jointgroupInput)
						{
							//no need to throw an error here, this happens in harmless situations. i'll just break out
							break;
							//throw "either linkgroupInput or jointgroupInput are null. this will fail, aborting";
						}
						linkselInput->clearSelection();
						jointselInput->clearSelection();
						//is it a link?
						ULink* currLink = dynamic_cast<ULink*>(_ms.thistree.currentEl);
						if (currLink)
						{
							std::vector<Ptr<Occurrence>> group = currLink->group;
							LOG(DEBUG) << "repopulating link selection";
							for (auto it = group.cbegin(); it != group.cend(); it++)
							{
								linkselInput->addSelection(*it);
							}
							//we hide the joint selection
							linkgroupInput->isVisible(true);
							jointgroupInput->isVisible(false);
						}
						//same for joints
						//ui->messageBox("ffs2");
						UJoint* currJoint = dynamic_cast<UJoint*>(_ms.thistree.currentEl);
						if (currJoint)
						{
							LOG(DEBUG) << "repopulating joint selection";
							jointselInput->addSelection(currJoint->entity);

							//we hide the link selection
							linkgroupInput->isVisible(false);
							jointgroupInput->isVisible(true);

							//we set the controls for parent and child links

							vector<string> alllinkgr = _ms.thistree.allLinksvec();

							if (!cln || !pln)
								throw "either cln or pln (or both!) don't exist. this will fail, aborting";

							cln->listItems()->clear();
							pln->listItems()->clear();

							cln->listItems()->add(">>not set<<", true);
							pln->listItems()->add(">>not set<<", true);

							LOG(DEBUG) << "repopulating joint names";
							for (auto linknamestr : alllinkgr)
							{
								bool isthischildselected = linknamestr == currJoint->childlink;
								bool isthisparentselected = linknamestr == currJoint->parentlink;
								cln->listItems()->add(linknamestr, isthischildselected);
								pln->listItems()->add(linknamestr, isthisparentselected);
							}

						}

					}
				}
				else
				{
					LOG(WARNING) << "could not resolve _ms.thistree.currentEl\nThis happens if there is no selected row and can usually be ignored.";
				}
			}
			catch (const char* msg) {
				LOG(ERROR) << msg;
				ui->messageBox(msg);
			}

			catch (std::exception& e)
			{
				LOG(ERROR) << e.what();
				std::string errormsg = "the update region bit failed...\n" ;
				LOG(ERROR) << errormsg;
				ui->messageBox(errormsg);
			}
			catch (...) // is there an exception that is not derived from std::exception?
			{
				std::string errormsg = "Error: the update region bit failed hard!";
				LOG(ERROR) << errormsg;
				ui->messageBox(errormsg);
			}
		}
	}
};

// CommandExecuted event handler.
class UrdfGenOnExecuteEventHander : public adsk::core::CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override
	{
		ui->messageBox("Executing! ");
		LOG(INFO) << "Executing! ";

		vector<fs::path> mypaths = createpaths(_ms.packagename, _ms.thisscriptpath);
		// lets create a simple xml to make sure we understand tinyxml sintax

		TiXmlDocument urdfdoc;

		//////change!!!
		TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
		urdfdoc.LinkEndChild(decl);

		TiXmlElement * robot_root = new TiXmlElement("robot");
		robot_root->SetAttribute("name", "gummi");
		urdfdoc.LinkEndChild(robot_root);

		//TiXmlText * text = new TiXmlText("Hello World!");
		//element->LinkEndChild(text);

		//ULink base_link;
		//base_link.makexml(robot_root, _ms.packagename);

		//UJoint testjoint;
		//testjoint.makexml(robot_root, _ms.packagename);

		ULink base_link;
		base_link.name = "base_link";
		//base_link = Link('base_link', -1)
		base_link.makexml(robot_root, _ms.packagename);
#
		UJoint setaxisjoint;
		setaxisjoint.name = "set_worldaxis";
		setaxisjoint.isset = true;
		setaxisjoint.type = "fixed";
		setaxisjoint.realorigin.rpy = std::to_string(PI / 2) + " 0 0";
		setaxisjoint.parentlink = "base_link";
		setaxisjoint.childlink = "base";
		setaxisjoint.makexml(robot_root, _ms.packagename);
		
		//now parse the urdftree
		for (auto el:_ms.thistree.elementsDict) 
		{
			ULink* currLink = dynamic_cast<ULink*>(el.second);
			if (currLink)
			{
				LOG(INFO) << "calling genlink for link:" + currLink->name;
				bool succeeded = currLink->genlink(mypaths[1], mypaths[2], design, app);
				if (!succeeded)
				{
					string errormsg = "failed generating link" + currLink->name;
					LOG(ERROR) << errormsg;
					ui->messageBox(errormsg);
					return;
				}
			}
			el.second->makexml(robot_root, _ms.packagename); //this should execute the derived class's makexml, i presume
		};

		string filenametosave = (mypaths[0] / "robot.urdf").string();
		//ui->messageBox(filenametosave); 
		LOG(INFO) << "Saving file" + (filenametosave); 
		urdfdoc.SaveFile(filenametosave.c_str());




		}
};

// CommandDestroyed event handler
class UrdfGenOnDestroyEventHandler : public adsk::core::CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override
	{
		//if i had logging I would need to stop it here. probably just need to reset _ms
		MotherShip _ms;
		(&_ms)->~MotherShip();
		new (&_ms) MotherShip();
		//adsk::terminate(); terminate will unload it. i want to keep unloading it to test it, but uncomment next line before commiting to master
		
		//if (!runfrommenu)
		LOG(INFO) << "Bye bye!";
		adsk::terminate();
	}
};

// Event handler for my custom validateInputs event.
class UrdfGenValidateInputsEventHandler : public adsk::core::ValidateInputsEventHandler
{
public:
	void notify(const Ptr<ValidateInputsEventArgs>& eventArgs) override
	{
		// Code to react to the event.

		//this event is actually triggered all the time. not sure how to use it. 
		//ui->messageBox("In UrdfGenValidateInputsEventHandler event handler.");

		// if the tree is empty, then just quit?
		LOG(DEBUG) << "Validating inputs";
	}
} _validateInputs;

// CommandCreated event handlers.
class UrdfGenCommandCreatedEventHandler : public adsk::core::CommandCreatedEventHandler
{
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override
	{
		if (eventArgs)
		{
			LOG(INFO) << "Hello from urdfgen. Creating GUI ";
			try 
			{
				//need to update default design!
				Ptr<Product> product = app->activeProduct();
				if (!product)
					throw "error: can't get active product!";

				//this feels silly but they are different objects. Cpp is very clear...
				design = product;
				if (!design)
					throw "can't get current design.";

				//
				// Creates our simple GUI
				//

				// Get the command that was created.
				Ptr<Command> command = eventArgs->command();
				if (command)
				{
					// Connect to the command destroyed event.
					Ptr<CommandEvent> onDestroy = command->destroy();
					if (!onDestroy)
						return;
					bool isOk = onDestroy->add(&onDestroyHandler);
					if (!isOk)
						return;

					// Connect to the input changed event.
					Ptr<InputChangedEvent> onInputChanged = command->inputChanged();
					if (!onInputChanged)
						return;
					isOk = onInputChanged->add(&onInputChangedHandler);
					if (!isOk)
						return;

					// Connect to the execute event.
					Ptr<CommandEvent> onExecute = command->execute();//execute or doexecute?
					if (!onInputChanged)
						return;
					isOk = onExecute->add(&onExecuteHandler);
					if (!isOk)
						return;

					//added later. trying for custom validation.
					// Connect the handler function to the event.
					Ptr<ValidateInputsEvent> validateInputsEvent = command->validateInputs();
					if (!validateInputsEvent)
						return;
					
					//I am not validating inputs right now; remove comments to use input validation callback.

					//isOk = validateInputsEvent->add(&_validateInputs);
					if (!isOk)
						return;

					// Get the CommandInputs collection associated with the command.
					Ptr<CommandInputs> inputs = command->commandInputs();
					if (!inputs)
						return;

					// Create a tab input.
					Ptr<TabCommandInput> tabCmdInput3 = inputs->addTabCommandInput("tab_1", "URDFGEN");
					if (!tabCmdInput3)
						return;
					Ptr<CommandInputs> tab3ChildInputs = tabCmdInput3->children();
					if (!tab3ChildInputs)
						return;

					tab3ChildInputs->addStringValueInput("packagename", "Name of your URDF package", _ms.packagename);

					// Create table input.
					Ptr<TableCommandInput> tableInput = tab3ChildInputs->addTableCommandInput("table", "Table", 3, "1:2:3:1");
					tableInput->maximumVisibleRows(20);
					tableInput->minimumVisibleRows(10);

					// Add inputs into the table.
					Ptr<CommandInput> addLinkButtonInput = tab3ChildInputs->addBoolValueInput("tableLinkAdd", "Add Link", false, "", true);
					tableInput->addToolbarCommandInput(addLinkButtonInput);
					Ptr<CommandInput> addJointButtonInput = tab3ChildInputs->addBoolValueInput("tableJointAdd", "Add Joint", false, "", true);
					tableInput->addToolbarCommandInput(addJointButtonInput);
					Ptr<CommandInput> deleteButtonInput = tab3ChildInputs->addBoolValueInput("tableDelete", "Delete", false, "", true);
					tableInput->addToolbarCommandInput(deleteButtonInput);

					tab3ChildInputs->addBoolValueInput("createtree", "Create tree", false, "", true);

					std::string messaged = "";
					tab3ChildInputs->addTextBoxCommandInput("debugbox", "", messaged, 10, true);

					// Adds a group for the link 

					Ptr<GroupCommandInput> linkGroupCmdInput = tab3ChildInputs->addGroupCommandInput("linkgroup", "Link Stuff");
					if (!linkGroupCmdInput)
						return;
					linkGroupCmdInput->isVisible(false);
					
					Ptr<CommandInputs> linkGroupChildInputs = linkGroupCmdInput->children();
					if (!linkGroupChildInputs)
						return;

					// Create a selection input for the link.
					Ptr<SelectionCommandInput> selectionInput1 = linkGroupChildInputs->addSelectionInput("linkselection", "Select Link Components", "Basic select command input");
					selectionInput1->addSelectionFilter("Occurrences");
					selectionInput1->setSelectionLimits(0);

					// Adds a group for the joint 
				
					Ptr<GroupCommandInput> jointGroupCmdInput = tab3ChildInputs->addGroupCommandInput("jointgroup", "Joint Stuff");
					if (!jointGroupCmdInput)
						return;
					jointGroupCmdInput->isVisible(false);
				
					Ptr<CommandInputs> jointGroupChildInputs = jointGroupCmdInput->children();
					if (!jointGroupChildInputs)
						return;

					// Create a selection input for the joint.
					Ptr<SelectionCommandInput> selectionInput2 = jointGroupChildInputs->addSelectionInput("jointselection", "Select Joint", "Basic select command input");
					selectionInput2->addSelectionFilter("Joints");
					selectionInput2->setSelectionLimits(0,1);
					selectionInput2->isVisible(true);

					//variant. doesn not set a single control, rather multiple ones and changes visibility.
					
					// Adds the joint offset control (maybe there is a built in version of this; the "move" command shows something like it)

					//JointControl jtctrl(jointGroupChildInputs);

					// Add parent and child controls for joint

					Ptr<DropDownCommandInput> parentlinkin = jointGroupChildInputs->addDropDownCommandInput("parentlinkname", "Name of parent link", DropDownStyles::TextListDropDownStyle);

					Ptr<DropDownCommandInput> childlinkin = jointGroupChildInputs->addDropDownCommandInput("childlinkname", "Name of child link", DropDownStyles::TextListDropDownStyle);
					LOG(INFO) << "Created GUI alright. ";
				}
			}
			catch (const char* msg) {
				LOG(ERROR) << msg;
				ui->messageBox(msg);
			}
			catch (...) //probably bad style. will just avoid failing here. 
			{
				const char* msg = "unknown error!?!";
				LOG(ERROR) << "unknown error!?!";
				ui->messageBox(msg);
			}
		}
	}
private:
	UrdfGenOnExecuteEventHander onExecuteHandler;
	UrdfGenOnDestroyEventHandler onDestroyHandler;
	UrdfGenOnInputChangedEventHander onInputChangedHandler;
} _urdfGenCmdCreatedHandler;

class GenSTLCreatedEventHandler : public adsk::core::CommandCreatedEventHandler
{
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override
	{
		if (eventArgs)
		{
			LOG(INFO) << "Hello from genSTL ";
			try
			{
				//need to update default design!
				Ptr<Product> product = app->activeProduct();
				if (!product)
					throw "error: can't get active product!";

				//this feels silly but they are different objects. Cpp is very clear...
				design = product;
				if (!design)
					throw "can't get current design.";

				Ptr<Component> rootComp = design->rootComponent();
				if (!rootComp)
					throw "error: can't find root component";
				//Ptr<OccurrenceList> allOccs = rootComp->allOccurrences();
				Ptr<ExportManager> exportMgr = design->exportManager();
				if (!exportMgr)
					throw "error: can't find export manager";

				Ptr<FileDialog> fileDlg = ui->createFileDialog();
				fileDlg->isMultiSelectEnabled(false);
				fileDlg->title("Choose location to save your STL ");
				fileDlg->filter("*.stl");
				DialogResults dlgResult = fileDlg->showSave();
				if (dlgResult != DialogResults::DialogOK)
					throw "You need to select a folder";
				
				Ptr<STLExportOptions> stlRootOptions = exportMgr->createSTLExportOptions(rootComp);
				if (!stlRootOptions)
					throw "error: can't set stl export options";
				stlRootOptions->filename(fileDlg->filename());
				stlRootOptions->sendToPrintUtility(false);
				exportMgr->execute(stlRootOptions);

				LOG(INFO) << "File " + fileDlg->filename() + " saved successfully";
				LOG(INFO)<< " genSTL succeeded!"; 
				
			}
			catch (const char* msg) {
				LOG(ERROR) << msg;
				ui->messageBox(msg);
			}
			catch (...) //probably bad style. will just avoid failing here. 
			{
				const char* msg = "unknown error!?!";
				LOG(ERROR) << "unknown error!?!";
				ui->messageBox(msg);
			}

		}
	}
private:
	// So yeah, I didn't create those because I don't need them for this simple command

	//OnExecuteEventHander onExecuteHandler;
	//OnDestroyEventHandler onDestroyHandler;
	//OnInputChangedEventHander onInputChangedHandler;
} _genSTLCmdCreatedHandler;

extern "C" XI_EXPORT bool run(const char* context)
{
	//actually, this dll is copied to the webdeploy path, so I would need to understand paths better 

	//_ms should exist already and have appdata and thisscriptpath set
	el::Configurations conf;// ((_ms.thisscriptpath / "inc" / "easylogging" / "conf").string());
	conf.setToDefault();
	conf.setGlobally(el::ConfigurationType::Filename, (_ms.thisscriptpath / "urdflog.log").string());
	el::Loggers::reconfigureLogger("default", conf); //configures easylogging with my config file..
	el::Loggers::reconfigureAllLoggers(conf);
	
	//let's use the cpp stuff
	
	LOG(INFO) << "Started!";
	app = Application::get();
	if (!app)
		return false;

	ui = app->userInterface();
	if (!ui)
		return false;

	//don't be dumb
	_ms.thistree.ui = ui;

	Ptr<Product> product = app->activeProduct();
	if (!product)
		return false;

	design = product;
	if (!design)
		return false;

	Ptr<ToolbarPanelList> toolBarPanels = ui->allToolbarPanels();
	if (!toolBarPanels)
		return false;

	Ptr<ToolbarPanel> tbPanel = toolBarPanels->itemById("SolidScriptsAddinsPanel");

	// Create the command definition.
	Ptr<CommandDefinitions> commandDefinitions = ui->commandDefinitions();
	if (!commandDefinitions)
		return nullptr;

	// Get the existing command definition or create it if it doesn't already exist.
	Ptr<CommandDefinition> urdfGenCmdDef = commandDefinitions->itemById("cmdInputsUrdfGen");
	if (!urdfGenCmdDef)
	{
		urdfGenCmdDef = commandDefinitions->addButtonDefinition("cmdInputsUrdfGen",
			"Make URDF",
			"My attempt to make an URDF from a Fusion model, only now in cpp.");
	}

	Ptr<CommandDefinition> genSTLcmdDef = commandDefinitions->itemById("cmdInputsgenSTL");
	if (!genSTLcmdDef)
	{
		genSTLcmdDef = commandDefinitions->addButtonDefinition("cmdInputsgenSTL",
			"Generate STL",
			"Generate single STL (in case some of them are incorrect/changed), only now in cpp.");
	}


	// Connect to the command created event.
	Ptr<CommandCreatedEvent> urdfGenCommandCreatedEvent = urdfGenCmdDef->commandCreated();
	if (!urdfGenCommandCreatedEvent)
		return false;
	urdfGenCommandCreatedEvent->add(&_urdfGenCmdCreatedHandler);

	Ptr<CommandCreatedEvent> genSTLcommandCreatedEvent = genSTLcmdDef->commandCreated();
	if (!genSTLcommandCreatedEvent)
		return false;
	genSTLcommandCreatedEvent->add(&_genSTLCmdCreatedHandler);

	Ptr<ToolbarControls> tbControls = tbPanel->controls();
	if (runfrommenu)
	{
		Ptr<ToolbarControl> aControl = tbControls->itemById("cmdInputsUrdfGen");
		while (aControl && aControl->isValid())
		{
			aControl->deleteMe();
			aControl = tbControls->itemById("cmdInputsUrdfGen");
		}
		Ptr<ToolbarControl> bControl = tbControls->itemById("cmdInputsgenSTL");
		while (bControl && bControl->isValid())
		{
			bControl->deleteMe();
			bControl = tbControls->itemById("cmdInputsgenSTL");
		}
		tbControls->addCommand(urdfGenCmdDef);
		tbControls->addCommand(genSTLcmdDef);
	}
	else
	{
		urdfGenCmdDef->execute();
	}

	//ui->messageBox("Hello addin");
	LOG(INFO) << "run finished";

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{
	LOG(INFO) << "Shutting down.";
	//removing buttons from toolbar
	Ptr<ToolbarPanelList> toolBarPanels = ui->allToolbarPanels();
	if (!toolBarPanels)
		return false;

	Ptr<ToolbarPanel> tbPanel = toolBarPanels->itemById("SolidScriptsAddinsPanel");

	Ptr<ToolbarControls> tbControls = tbPanel->controls();
	if (runfrommenu)
	{
		Ptr<ToolbarControl> aControl = tbControls->itemById("cmdInputsUrdfGen");
		while (aControl && aControl->isValid())
		{
			aControl->deleteMe();
			aControl = tbControls->itemById("cmdInputsUrdfGen");
		}
		Ptr<ToolbarControl> bControl = tbControls->itemById("cmdInputsgenSTL");
		while (bControl && bControl->isValid())
		{
			bControl->deleteMe();
			bControl = tbControls->itemById("cmdInputsgenSTL");
		}

	}

	if (ui)
	{
		//ui->messageBox("Stop addin");
		ui = nullptr;
	}
	LOG(INFO) << "stop finished";
	return true;
}


#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
