
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

//UI bits:

// InputChange event handler.
class UrdfGenOnInputChangedEventHander : public adsk::core::InputChangedEventHandler
{
public:
	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override
	{
		
	}
};

// CommandExecuted event handler.
class UrdfGenOnExecuteEventHander : public adsk::core::CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override
	{

	}
};

// CommandDestroyed event handler
class UrdfGenOnDestroyEventHandler : public adsk::core::CommandEventHandler
{
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override
	{
		adsk::terminate();
	}
};

// CommandCreated event handler.
class UrdfGenCommandCreatedEventHandler : public adsk::core::CommandCreatedEventHandler
{
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override
	{
		if (eventArgs)
		{
			ui->messageBox("Hello from urdfgen ");
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
