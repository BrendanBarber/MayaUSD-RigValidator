#include "ValidateRigCmd.h"

#include <memory>
#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MFnDagNode.h>
#include <maya/MDagPath.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMatrixData.h>
#include <maya/MItGeometry.h>
#include <maya/MPlug.h>
#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MDagPathArray.h>
#include <maya/MMatrix.h>
#include <maya/MFnIkJoint.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usdSkel/skeleton.h>
#include <pxr/usd/usdSkel/topology.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>

const char* ValidateRigCmd::commandName = "validateRig";

const char* ValidateRigCmd::rootFlag = "-r";
const char* ValidateRigCmd::rootFlagLong = "-root";
const char* ValidateRigCmd::pathFlag = "-u";
const char* ValidateRigCmd::pathFlagLong = "-usdFile";

ValidateRigCmd::ValidateRigCmd() {
	ValidateRigCmd::m_usdFilePath = "";
}

ValidateRigCmd::~ValidateRigCmd() {
	
}

void* ValidateRigCmd::creator() {
	return new ValidateRigCmd();
}

MSyntax ValidateRigCmd::newSyntax() {
	MSyntax syntax;
	syntax.addFlag(rootFlag, rootFlagLong, MSyntax::kString);
	syntax.addFlag(pathFlag, pathFlagLong, MSyntax::kString);

	return syntax;
}

MStatus ValidateRigCmd::doIt(const MArgList& arg) {
	return MS::kSuccess;
}

MStatus ValidateRigCmd::redoIt() {
	return MS::kSuccess;
}

MStatus ValidateRigCmd::undoIt() {
	return MS::kSuccess;
}

bool ValidateRigCmd::isUndoable() const {
	return false;
}

std::unique_ptr<ValidateRigCmd::USDSkeletonData> ValidateRigCmd::parseUSDSkelData(const MString& filePath, const SdfPath& skelPath)
{
	UsdStageRefPtr stage = UsdStage::Open(filePath.asChar(), UsdStage::LoadAll);
	if (!stage) {
		MGlobal::displayError("Failed to open USD file: " + filePath);
		return nullptr;
	}

	// Get the skeleton prim
	UsdPrim skelPrim = stage->GetPrimAtPath(skelPath);
	if (!skelPrim.IsValid()) {
		MGlobal::displayError("Invalid skeleton path: " + MString(skelPath.GetText()));
		return nullptr;
	}

	// Create UsdSkelSkeleton shema
	UsdSkelSkeleton skeleton(skelPrim);
	if (!skeleton) {
		MGlobal::displayError("Prim is not a valid UsdSkelSkeleton: " + MString(skelPath.GetText()));
		return nullptr;
	}

	// Create the USD data structure
	auto skelData = std::make_unique<USDSkeletonData>();
	skelData->primPath = skelPath;

	// Extract joint names
	UsdAttribute jointsAttr = skeleton.GetJointsAttr();
	if (!jointsAttr.Get(&skelData->jointNames)) {
		MGlobal::displayError("Failed to read joints attribute");
		return nullptr;
	}

	// Extract joint parent indices
	UsdSkelTopology topology(skelData->jointNames);
	VtIntArray parentIndices = topology.GetParentIndices();
	skelData->jointParentIndices = parentIndices;

	// Extract bind transforms
	UsdAttribute bindTransformsAttr = skeleton.GetBindTransformsAttr();
	if (!bindTransformsAttr.Get(&skelData->bindTransforms)) {
		MGlobal::displayError("Failed to read bind transforms");
		return nullptr;
	}

	// Extract rest transforms
	UsdAttribute restTransformsAttr = skeleton.GetRestTransformsAttr();
	if (!restTransformsAttr.Get(&skelData->restTransforms)) {
		MGlobal::displayError("Failed to read rest transforms");
		return nullptr;
	}

	// Validate data
	size_t numJoints = skelData->jointNames.size();
	if (skelData->jointParentIndices.size() != numJoints ||
		skelData->bindTransforms.size() != numJoints ||
		skelData->restTransforms.size() != numJoints) {
		MGlobal::displayError("Inconsistent skeleton data sizes");
		return nullptr;
	}

	return skelData;
}

std::vector<ValidateRigCmd::USDSkeletonData> ValidateRigCmd::parseAllUSDSkels(const MString& filePath)
{
	std::vector<USDSkeletonData> skeletons;

	// Open the USD stage
	UsdStageRefPtr stage = UsdStage::Open(filePath.asChar());
	if (!stage) {
		MGlobal::displayError("Failed to open USD file: " + filePath);
		return skeletons;
	}

	// Traverse the stage to find all UsdSkelSkeleton prims
	UsdPrimRange primRange = stage->Traverse();
	for (UsdPrim prim : primRange) {
		if (prim.IsA<UsdSkelSkeleton>()) {
			SdfPath skelPath = prim.GetPath();

			// Parse this skeleton
			auto skelData = parseUSDSkelData(filePath, skelPath);
			if (skelData) {
				skeletons.push_back(std::move(*skelData));
			}
			else {
				MGlobal::displayWarning("Failed to parse skeleton at path: " +
					MString(skelPath.GetText()));
			}
		}
	}

	if (skeletons.empty()) {
		MGlobal::displayWarning("No UsdSkelSkeleton prims found in file: " + filePath);
	}
	else {
		MGlobal::displayInfo(MString("Found ") + skeletons.size() +
			" skeleton(s) in file: " + filePath);
	}

	return skeletons;
}

std::unique_ptr<ValidateRigCmd::MayaSkeletonData> ValidateRigCmd::parseMayaSkel(const MDagPath& root)
{
	MStatus status;

	// Verify that root is a joint
	if (!root.hasFn(MFn::kJoint)) {
		MGlobal::displayError("Root path is not a joint");
		return nullptr;
	}

	auto skelData = std::make_unique<MayaSkeletonData>();
	skelData->rootPath = root;

	std::vector<MDagPath> jointPaths;
	std::map<std::string, int> jointNameToIndex;

	// Recursively traverse the joint hierarchy
	std::function<void(const MDagPath&)> traverseJoints = [&](const MDagPath dagPath) {
		MFnDagNode dagNode(dagPath, &status);
		if (status != MS::kSuccess) return;

		// Add joint
		int currentIndex = jointPaths.size();
		jointPaths.push_back(dagPath);
		jointNameToIndex[dagPath.partialPathName().asChar()] = currentIndex;

		// Traverse children
		for (unsigned int i = 0; i < dagNode.childCount(); ++i) {
			MObject child = dagNode.child(i, &status);
			if (status == MS::kSuccess && child.hasFn(MFn::kJoint)) {
				MDagPath childPath = dagPath;
				childPath.push(child);
				traverseJoints(childPath);
			}
		}
	};

	// Build joint hierarchy
	traverseJoints(root);

	if (jointPaths.empty()) {
		MGlobal::displayError("No joints found in hierarchy");
		return nullptr;
	}

	// Extract data for each joint
	for (size_t i = 0; i < jointPaths.size(); ++i) {
		const MDagPath& jointPath = jointPaths[i];
		MFnIkJoint joint(jointPath, &status);
		if (status != MS::kSuccess) {
			MGlobal::displayError("Failed to create MFnIkJoint for: " + jointPath.partialPathName());
			return nullptr;
		}

		// Joint name
		skelData->jointNames.append(jointPath.partialPathName());

		// Parent index
		int parentIndex = -1;
		if (jointPath.length() > 1) {
			MDagPath parentPath = jointPath;
			parentPath.pop();
			if (parentPath.hasFn(MFn::kJoint)) {
				auto it = jointNameToIndex.find(parentPath.partialPathName().asChar());
				if (it != jointNameToIndex.end()) {
					parentIndex = it->second;
				}
			}
		}
		skelData->jointParentIndices.append(parentIndex);

		// Rest transform (local transform)
		MTransformationMatrix restXform = joint.transformation(&status);
		if (status != MS::kSuccess) {
			MGlobal::displayError("Failed to get transformation for: " + jointPath.partialPathName());
			return nullptr;
		}
		skelData->restTransforms.append(restXform.asMatrix());

		// Bind transform (inverse bind matrix from skin cluster)
		// TODO: Query the skin cluster's bind pre-matrix
		MMatrix worldMatrix = jointPath.inclusiveMatrix(&status);
		if (status != MS::kSuccess) {
			MGlobal::displayError("Failed to get world matrix for: " + jointPath.partialPathName());
			return nullptr;
		}
		skelData->bindTransforms.append(worldMatrix);
	}

	MGlobal::displayInfo(MString("Parsed Maya skeleton with ") +
		skelData->jointNames.length() + " joints");

	return skelData;
}

std::unique_ptr<ValidateRigCmd::MayaSkinBindingData> ValidateRigCmd::parseMayaSkin(const MDagPath& meshPath)
{
	MStatus status;
	auto data = std::make_unique<MayaSkinBindingData>();

	data->geomPath = meshPath;

	MObject meshObj = meshPath.node();
	MItDependencyNodes itDep(MFn::kSkinClusterFilter);

	MObject skinClusterObj;
	bool foundSkinCluster = false;

	while (!itDep.isDone()) {
		MObject currentObj = itDep.thisNode();
		MFnSkinCluster skinCluster(currentObj, &status);

		if (status == MS::kSuccess) {
			// Check if this skin cluster is connected to our mesh
			unsigned int numGeoms = skinCluster.numOutputConnections();
			for (unsigned int i = 0; i < numGeoms; i++) {
				unsigned int index = skinCluster.indexForOutputConnection(i, &status);
				MDagPath outputPath;
				skinCluster.getPathAtIndex(index, outputPath);

				if (outputPath == meshPath) {
					skinClusterObj = currentObj;
					foundSkinCluster = true;
					break;
				}
			}
		}

		if (foundSkinCluster) break;
		itDep.next();
	}

	if (!foundSkinCluster) {
		MGlobal::displayWarning("No skin cluster found for mesh: " + meshPath.fullPathName());
		return nullptr;
	}

	MFnSkinCluster skinCluster(skinClusterObj, &status);
	CHECK_MSTATUS_AND_RETURN(status, nullptr);

	// Get the skeleton root path
	MDagPathArray influencePaths;
	unsigned int numInfluences = skinCluster.influenceObjects(influencePaths, &status);
	CHECK_MSTATUS_AND_RETURN(status, nullptr);

	if (numInfluences == 0) {
		MGlobal::displayWarning("Skin cluster has no influence objects");
		return nullptr;
	}

	// Common root for all influences
	data->skelPath = influencePaths[0];
	while (data->skelPath.length() > 0) {
		bool isCommonRoot = true;
		MString rootPathStr = data->skelPath.fullPathName();

		for (unsigned int i = 1; i < numInfluences; i++) {
			MString influencePathStr = influencePaths[i].fullPathName();
			// Check if influence path starts with root path
			if (influencePathStr.indexW(rootPathStr) != 0) {
				isCommonRoot = false;
				break;
			}
		}
		if (isCommonRoot) break;
		data->skelPath.pop();
	}

	// Geo bind transorm
	MFnTransform geomTransform(meshPath, &status);
	if (status == MS::kSuccess) {
		// Get the world matrix at bind time
		MPlug bindPreMatrixPlug = skinCluster.findPlug("bindPreMatrix", true, &status);
		if (status == MS::kSuccess && bindPreMatrixPlug.isArray()) {
			MPlug elementPlug = bindPreMatrixPlug.elementByLogicalIndex(0, &status);
			if (status == MS::kSuccess) {
				MObject matrixData;
				elementPlug.getValue(matrixData);
				MFnMatrixData matrixFn(matrixData);
				data->geomBindTransform = matrixFn.matrix();
			}
		}
	}

	// Get vertex weights and joint indices
	MItGeometry geoIter(meshPath);
	unsigned int vertexCount = geoIter.count();

	// Preallocate arrays, estimate 4 influences per vertex
	data->jointIndices.setLength(vertexCount * 4);
	data->jointWeights.setLength(vertexCount * 4);

	unsigned int arrayIndex = 0;

	for (; !geoIter.isDone(); geoIter.next()) {
		MObject component = geoIter.currentItem();

		// Get weights for this vertex
		MDoubleArray weights;
		unsigned int influenceCount;
		skinCluster.getWeights(meshPath, component, weights, influenceCount);

		// Store non-zero weights
		for (unsigned int i = 0; i < influenceCount; i++) {
			if (weights[i] > 0.0001) {  // Threshold to skip negligible weights
				// Make sure we have space
				if (arrayIndex >= data->jointIndices.length()) {
					data->jointIndices.setLength(data->jointIndices.length() + vertexCount);
					data->jointWeights.setLength(data->jointWeights.length() + vertexCount);
				}

				data->jointIndices[arrayIndex] = i;
				data->jointWeights[arrayIndex] = static_cast<float>(weights[i]);
				arrayIndex++;
			}
		}
	}

	// Trim arrays to actual size
	data->jointIndices.setLength(arrayIndex);
	data->jointWeights.setLength(arrayIndex);

	return data;
}
