
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#include "inc/tinyxml.h"
#include "urdftree.h"

//do I need these or is the reference guide outdated?
#include <Core/UserInterface/Command.h>
#include <Core/UserInterface/ValidateInputsEvent.h>
#include <Core/UserInterface/ValidateInputsEventHandler.h>
#include <Core/UserInterface/ValidateInputsEventArgs.h>

using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

Ptr<Application> app;
Ptr<UserInterface> ui;
Ptr<Design> design;

const bool runfrommenu = true; // this allowed to be run as script as well. TODO: KEEP? 




class MotherShip
{
public:
	int rowNumber = 0, elnum = 0, oldrow = -1, numlinks = -1, numjoints = -1;
	std::string packagename = "mypackage";
	//missing! jtctrl lastjoint is maybe not a ujoint object?
	UJoint lastjoint;
	UrdfTree thistree;
	void setCurrEl() {};
	MotherShip() {};
	~MotherShip() {};
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

void UrdfTree::rmElement(int elnum)
{
	elementsDict.erase(elementsDict.begin() + elnum);
	_ms.rowNumber -= 1;
};

//////////////////////////////////////////////////////
//UI bits:
//////////////////////////////////////////////////////

//Adds element to tree, i.e., row to table

void addRowToTable(Ptr<TableCommandInput> tableInput, std::string LinkOrJoint)
{
	bool islink;
	std::string elname;
	if (!tableInput)
		return;
	// Get the CommandInputs object associated with the parent command.
	Ptr<CommandInputs> cmdInputs = tableInput->commandInputs();

	// Setting up controls to be added for each row
	Ptr<CommandInput> elnnumInput = cmdInputs->addStringValueInput("elnum" + std::to_string(_ms.elnum), "elnumTable" + std::to_string(_ms.elnum), std::to_string(_ms.elnum));
	elnnumInput->isEnabled(false);

	if (LinkOrJoint == "" || LinkOrJoint == "Link")
	{
		islink = true;
		_ms.numlinks += 1;
		if (_ms.elnum == 0)
			elname = "base";
		else
			elname = "link" + std::to_string(_ms.numlinks);
	}
	else if (LinkOrJoint == "Joint")
	{
		islink = false;
		_ms.numjoints += 1;
		elname = "joint" + std::to_string(_ms.numjoints);
	}
	Ptr<DropDownCommandInput> JorLInput = cmdInputs->addDropDownCommandInput("TableInput_value" + std::to_string(_ms.elnum), "JorLTable" + std::to_string(_ms.elnum), DropDownStyles::TextListDropDownStyle);
	Ptr<ListItems> dropdownItems = JorLInput->listItems();
	if (!dropdownItems)
		return;
	dropdownItems->add("Link", islink, "");
	dropdownItems->add("Joint", !islink, "");

	Ptr<CommandInput> stringInput = cmdInputs->addStringValueInput("TableInput_string" + std::to_string(_ms.elnum), "StringTable" + std::to_string(_ms.elnum), elname);
	stringInput->isEnabled(false); //I'm disabling the ability to change element's name randomly...

	Ptr<CommandInput> slbutInput = cmdInputs->addBoolValueInput("butselectClick" + std::to_string(_ms.elnum), "Select", false, "", true);

    // Add the inputs to the table.
	int row = tableInput->rowCount();
	tableInput->addCommandInput(elnnumInput, row, 0);
	tableInput->addCommandInput(JorLInput, row, 1);
	tableInput->addCommandInput(stringInput, row, 2);
	tableInput->addCommandInput(slbutInput, row, 2);

	// Increment a counter used to make each row unique.

	_ms.rowNumber = _ms.rowNumber + 1;
	_ms.elnum += 1;

};

// InputChange event handler.
class UrdfGenOnInputChangedEventHander : public adsk::core::InputChangedEventHandler
{
public:
	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override
	{
		std::string jointname;
		Ptr<CommandInputs> inputs = eventArgs->inputs();
		if (!inputs)
			return;

		Ptr<CommandInput> cmdInput = eventArgs->input();
		if (!cmdInput)
			return;

		Ptr<TableCommandInput> tableInput = inputs->itemById("table");
		if (!tableInput)
			return;

		Ptr<TextBoxCommandInput> debugInput = inputs->itemById("debugbox");

		//define the groups first:
		Ptr<GroupCommandInput> linkgroupInput = inputs->itemById("linkgroup");
		Ptr<GroupCommandInput> jointgroupInput = inputs->itemById("jointgroup");

		Ptr<SelectionCommandInput> linkselInput;
		Ptr<SelectionCommandInput> jointselInput;
		//inside the group, there is no group!
		if (!linkgroupInput) // linkgroupInput is None:			
			linkselInput = inputs->itemById("linkselection");
		else
			linkselInput = linkgroupInput->children()->itemById("linkselection");

		//inside the group, there is no group!
		if (!jointgroupInput) // linkgroupInput is None:			
			jointselInput = inputs->itemById("jointselection");
		else
			jointselInput = jointgroupInput->children()->itemById("jointselection");

		if (cmdInput->id() == "tableLinkAdd") {
			addRowToTable(tableInput, "Link");
		}
		else if (cmdInput->id() == "tableJointAdd") {
			addRowToTable(tableInput, "Joint");
			tableInput->getInputAtPosition(_ms.rowNumber - 1, 1)->isEnabled(false);
			Ptr<StringValueCommandInput> thisstringinput = tableInput->getInputAtPosition(_ms.rowNumber - 1, 2);
			jointname = thisstringinput->value();
			_ms.thistree.addJoint(jointname, _ms.elnum - 1);
		}
		else if (cmdInput->id() == "tableDelete") {
			if (tableInput->selectedRow() == -1) {
				ui->messageBox("Select one row to delete.");
			}
			else {
				tableInput->deleteRow(tableInput->selectedRow());
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
		//adsk::terminate(); terminate will unload it. i want to keep unloading it to test it, but uncomment next line before commiting to master
		
		//if (!runfrommenu)
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
	}
} _validateInputs;

// CommandCreated event handler.
class UrdfGenCommandCreatedEventHandler : public adsk::core::CommandCreatedEventHandler
{
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override
	{
		if (eventArgs)
		{
			ui->messageBox("Hello from urdfgen ");
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

					isOk = validateInputsEvent->add(&_validateInputs);
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
					linkGroupCmdInput->isVisible(true);

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
					jointGroupCmdInput->isVisible(true);
				
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
				}
			}
			catch (const char* msg) {
				ui->messageBox(msg);
			}
			catch (...) //probably bad style. will just avoid failing here. 
			{
				ui->messageBox("unknown error!?!");
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
			ui->messageBox("Hello from genSTL ");
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

				ui->messageBox("File " + fileDlg->filename() + " saved successfully");
				
			}
			catch (const char* msg) {
				ui->messageBox(msg);
			}
			catch (...) //probably bad style. will just avoid failing here. 
			{
				ui->messageBox("unknown error!?!");
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
	app = Application::get();
	if (!app)
		return false;

	ui = app->userInterface();
	if (!ui)
		return false;

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


	ui->messageBox("Hello addin");

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{

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
		ui->messageBox("Stop addin");
		ui = nullptr;
	}

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
