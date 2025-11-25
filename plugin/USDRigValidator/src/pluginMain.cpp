#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj)
{
	const char* pluginVendor = "Brendan Barber";
	const char* pluginVersion = "0.1";

	MFnPlugin fnPlugin(obj, pluginVendor, pluginVersion);

	MGlobal::displayInfo("Plugin has been initialized!");

	return (MS::kSuccess);
}

MStatus uninitializePlugin(MObject obj)
{
	MGlobal::displayInfo("Plugin has been uninitialized!");

	return (MS::kSuccess);
}