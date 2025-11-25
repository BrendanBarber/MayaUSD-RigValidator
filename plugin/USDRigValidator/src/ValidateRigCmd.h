#pragma once

#include <vector>
#include <maya/MPxCommand.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MStringArray.h>
#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>
#include <maya/MMatrixArray.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/matrix4d.h>

PXR_NAMESPACE_USING_DIRECTIVE

class ValidateRigCmd : public MPxCommand 
{
public:
	static const char* commandName;

	ValidateRigCmd();
	virtual ~ValidateRigCmd();

	virtual MStatus doIt(const MArgList& arg) override;
	virtual MStatus redoIt() override;
	virtual MStatus undoIt() override;
	virtual bool isUndoable() const override;

	static void* creator();

	static MSyntax newSyntax();

	struct USDSkeletonData {
		SdfPath primPath;
		VtTokenArray jointNames;
		VtArray<int> jointParentIndices;
		VtArray<GfMatrix4d> bindTransforms;
		VtArray<GfMatrix4d> restTransforms;
	};

	struct USDSkinBindingData {
		SdfPath skelPath;
		SdfPath geomPath;
		VtArray<int> jointIndices;
		VtArray<float> jointWeights;
		GfMatrix4d geomBindTransform;
	};

	struct MayaSkeletonData {
		MDagPath rootPath;
		MStringArray jointNames;
		MIntArray jointParentIndices;
		MMatrixArray bindTransforms;
		MMatrixArray restTransforms;
	};

	struct MayaSkinBindingData {
		MDagPath skelPath;
		MDagPath geomPath;
		MIntArray jointIndices;
		MFloatArray jointWeights;
		MMatrix geomBindTransform;
	};

private:
	static const char* rootFlag;
	static const char* rootFlagLong;
	static const char* pathFlag;
	static const char* pathFlagLong;

	MDagPath m_root;
	MString m_usdFilePath;

	std::unique_ptr<USDSkeletonData> parseUSDSkelData(const MString& filePath, const SdfPath& skelPath);
	std::vector<USDSkeletonData> parseAllUSDSkels(const MString& filePath);
	std::unique_ptr<MayaSkeletonData> parseMayaSkel(const MDagPath& root);
	std::unique_ptr<MayaSkinBindingData> parseMayaSkin(const MDagPath& meshPath);
};