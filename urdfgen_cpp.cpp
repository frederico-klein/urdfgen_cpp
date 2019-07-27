
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>


using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

Ptr<Application> app;
Ptr<UserInterface> ui;
Ptr<Design> design;

const bool runfrommenu = true; // this allowed to be run as script as well. TODO: KEEP? 

const double PI = 3.14159265359;

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


	ui->messageBox("Hello addin");

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{
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
