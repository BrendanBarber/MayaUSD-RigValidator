#include "ValidateRigCmd.h"

#include <memory>
#include <cstdlib>
#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MFnDagNode.h>
#include <maya/MDagPath.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MFnDependencyNode.h>
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

	// Get root world matrix, used for rest transform calculation
	MMatrix rootWorldMatrix = root.inclusiveMatrix(&status);
	if (status != MS::kSuccess) {
		MGlobal::displayError("Failed to get root world matrix");
		return nullptr;
	}
	MMatrix rootWorldInverse = rootWorldMatrix.inverse();

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

		// Rest transform
		MMatrix jointWorldMatrix = jointPath.inclusiveMatrix(&status);
		if (status != MS::kSuccess) {
			MGlobal::displayError("Failed to get world matrix for: " + jointPath.partialPathName());
			return nullptr;
		}
		MMatrix skelSpaceRestMatrix = jointWorldMatrix * rootWorldInverse;
		skelData->restTransforms.append(skelSpaceRestMatrix);

		// Bind transform (inverse bind matrix from skin cluster)
		MMatrix bindMatrix = getBindMatrixForJoint(jointPath, status);
		if (status != MS::kSuccess) {
			MGlobal::displayError("Failed to get bind matrix for: " + jointPath.partialPathName());
			return nullptr;
		}
		skelData->bindTransforms.append(bindMatrix);
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

bool ValidateRigCmd::quickValidateSkeleton(const USDSkeletonData& usdSkel, const MayaSkeletonData& mayaSkel) 
{
	// Fastest checks
	if (usdSkel.jointNames.size() != mayaSkel.jointNames.length()) return false;
	if (usdSkel.jointParentIndices.size() != mayaSkel.jointParentIndices.length()) return false;
	if (usdSkel.bindTransforms.size() != mayaSkel.bindTransforms.length()) return false;

	// Slower checks
	for (size_t i = 0; i < usdSkel.jointNames.size(); ++i) {
		if (usdSkel.jointNames[i] != mayaSkel.jointNames[i].asChar())
			return false;
	}

	for (size_t i = 0; i < usdSkel.jointParentIndices.size(); ++i) {
		if (usdSkel.jointParentIndices[i] != mayaSkel.jointParentIndices[i])
			return false;
	}

	// Slowest checks
	for (size_t i = 0; i < usdSkel.bindTransforms.size(); ++i) {
		if (!matricesMatch(usdSkel.bindTransforms[i], mayaSkel.bindTransforms[i]))
			return false;
	}

	for (size_t i = 0; i < usdSkel.restTransforms.size(); ++i) {
		if (!matricesMatch(usdSkel.restTransforms[i], mayaSkel.restTransforms[i]))
			return false;
	}

	return true;
}

std::vector<ValidateRigCmd::ValidationIssue> ValidateRigCmd::detailedValidateSkeleton(
	const USDSkeletonData& usdSkel, const MayaSkeletonData& mayaSkel
)
{
	std::vector<ValidationIssue> issues;

	// Joint count
	if (usdSkel.jointNames.size() != mayaSkel.jointNames.length()) {
		MString desc;
		desc.format("Joint count mismatch: USD has ^1s joints, Maya has ^2s joints",
			MString() + (int)usdSkel.jointNames.size(),
			MString() + mayaSkel.jointNames.length());
		issues.emplace_back(ValidationIssue::Type::JOINT_COUNT_MISMATCH, desc);
		
		return issues; // For loops later won't work with number mismatch, return early 
	}

	// Joint names
	for (size_t i = 0; i < usdSkel.jointNames.size(); ++i) {
		if (usdSkel.jointNames[i] != mayaSkel.jointNames[i].asChar()) {
			MString desc;
			desc.format("Joint ^1s name mismatch: USD=''^2s'', Maya=''^3s''",
				MString() + (int)i,
				MString(usdSkel.jointNames[i].GetString().c_str()),
				mayaSkel.jointNames[i]);
			issues.emplace_back(ValidationIssue::Type::JOINT_NAME_MISMATCH, desc, i);
		}
	}

	// Parent indices
	for (size_t i = 0; i < usdSkel.jointParentIndices.size(); ++i) {
		if (usdSkel.jointParentIndices[i] != mayaSkel.jointParentIndices[i]) {
			MString desc;
			desc.format("Joint ^1s parent index mismatch: USD=^2s, Maya^3s",
				MString() + (int)i,
				MString() + usdSkel.jointParentIndices[i],
				MString() + mayaSkel.jointParentIndices[i]);
			issues.emplace_back(ValidationIssue::Type::PARENT_INDEX_MISMATCH, desc, i);
		}
	}

	// Bind Transforms
	for (size_t i = 0; i < usdSkel.bindTransforms.size(); ++i) {
		if (!matricesMatch(usdSkel.bindTransforms[i], mayaSkel.bindTransforms[i])) {
			MString desc;
			desc.format("Joint ^1s (^2s) bind transform mismatch",
				MString() + (int)i,
				mayaSkel.jointNames[i]);
			issues.emplace_back(ValidationIssue::Type::BIND_TRANSFORM_MISMATCH, desc, i);
		}
	}
	
	// Rest transforms
	for (size_t i = 0; i < usdSkel.restTransforms.size(); ++i) {
		if (!matricesMatch(usdSkel.restTransforms[i], mayaSkel.restTransforms[i])) {
			MString desc;
			desc.format("Joint ^1s (^2s) rest transform mismatch",
				MString() + (int)i,
				mayaSkel.jointNames[i]);
			issues.emplace_back(ValidationIssue::Type::REST_TRANSFORM_MISMATCH, desc, i);
		}
	}

	return issues;
}

bool ValidateRigCmd::quickValidateSkinBinding(
	const USDSkinBindingData& usdSkin,
	const MayaSkinBindingData& mayaSkin) 
{
	// Quick checks
	if (usdSkin.jointIndices.size() != mayaSkin.jointIndices.length()) return false;

	if (usdSkin.jointWeights.size() != mayaSkin.jointWeights.length()) return false;

	// Joint indices
	for (size_t i = 0; i < usdSkin.jointIndices.size(); ++i) {
		if (usdSkin.jointIndices[i] != mayaSkin.jointIndices[i]) return false;
	}

	// Joint weights
	const float weightTolerance = 1e-5f;
	for (size_t i = 0; i < usdSkin.jointWeights.size(); ++i) {
		float diff = std::abs(usdSkin.jointWeights[i] - mayaSkin.jointWeights[i]);
		if (diff > weightTolerance) {
			return false;
		}
	}

	// Geometry bind transform
	if (!matricesMatch(usdSkin.geomBindTransform, mayaSkin.geomBindTransform)) return false;

	return true;
}

std::vector<ValidateRigCmd::ValidationIssue> ValidateRigCmd::detailedValidateSkinBinding(
	const USDSkinBindingData& usdSkin,
	const MayaSkinBindingData& mayaSkin
)
{
	std::vector<ValidationIssue> issues;

	// Joint indices count
	if (usdSkin.jointIndices.size() != mayaSkin.jointIndices.length()) {
		MString desc;
		desc.format("Joint indices count mismatch: USD has ^1s, Maya has ^2s",
			MString() + (int)usdSkin.jointIndices.size(),
			MString() + mayaSkin.jointIndices.length());
		issues.emplace_back(ValidationIssue::Type::WEIGHT_COUNT_MISMATCH, desc);
		return issues; // For loops later won't work with number mismatch, return early 
	}

	// Joint weight count
	if (usdSkin.jointWeights.size() != mayaSkin.jointWeights.length()) {
		MString desc;
		desc.format("Joint weights count mismatch: USD has ^1s, Maya has ^2s",
			MString() + (int)usdSkin.jointWeights.size(),
			MString() + mayaSkin.jointWeights.length());
		issues.emplace_back(ValidationIssue::Type::WEIGHT_COUNT_MISMATCH, desc);
		return issues; // For loops later won't work with number mismatch, return early 
	}

	// Compare joint indices
	int indicesMismatchCount = 0;
	for (size_t i = 0; i < usdSkin.jointIndices.size(); ++i) {
		if (usdSkin.jointIndices[i] != mayaSkin.jointIndices[i]) {
			indicesMismatchCount++;
			// Only report first 5 mismatches
			if (indicesMismatchCount <= 5) {
				MString desc;
				desc.format("Joint index mismatch at position ^1s: USD=^2s, Maya=^3s",
					MString() + (int)i,
					MString() + usdSkin.jointIndices[i],
					MString() + mayaSkin.jointIndices[i]);
				issues.emplace_back(ValidationIssue::Type::JOINT_INDEX_MISMATCH, desc, i);
			}
		}
	}
	if (indicesMismatchCount > 5) {
		MString desc;
		desc.format("... and ^1s more joint index mismatches (showing first 5 only)",
			MString() + (indicesMismatchCount - 5));
		issues.emplace_back(ValidationIssue::Type::JOINT_INDEX_MISMATCH, desc);
	}

	// Compare joint weights
	const float weightTolerance = 1e-5f;
	int weightsMismatchCount = 0;
	for (size_t i = 0; i < usdSkin.jointWeights.size(); ++i) {
		float diff = std::abs(usdSkin.jointWeights[i] - mayaSkin.jointWeights[i]);
		if (diff > weightTolerance) {
			weightsMismatchCount++;
			// Only report first 5 mismatches to avoid spam
			if (weightsMismatchCount <= 5) {
				MString desc;
				desc.format("Weight mismatch at position ^1s: USD=^2s, Maya=^3s (diff=^4s)",
					MString() + (int)i,
					MString() + usdSkin.jointWeights[i],
					MString() + mayaSkin.jointWeights[i],
					MString() + diff);
				issues.emplace_back(ValidationIssue::Type::WEIGHT_VALUE_MISMATCH, desc, i);
			}
		}
	}
	if (weightsMismatchCount > 5) {
		MString desc;
		desc.format("... and ^1s more weight mismatches (showing first 5 only)",
			MString() + (weightsMismatchCount - 5));
		issues.emplace_back(ValidationIssue::Type::WEIGHT_VALUE_MISMATCH, desc);
	}

	// Geometry bind transform
	if (!matricesMatch(usdSkin.geomBindTransform, mayaSkin.geomBindTransform)) {
		MString desc("Geometry bind transform mismatch");
		issues.emplace_back(ValidationIssue::Type::GEOM_BIND_TRANSFORM_MISMATCH, desc);
	}

	return issues;
}

bool ValidateRigCmd::matricesMatch(const GfMatrix4d& usdMat,
	const MMatrix& mayaMat,
	double tolerance)
{
	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 4; ++col) {
			double diff = std::abs(usdMat[row][col] - mayaMat(row, col));
			if (diff > tolerance)
				return false;
		}
	}
	return true;
}

MMatrix ValidateRigCmd::getBindMatrixForJoint(const MDagPath& jointPath, MStatus& status)
{
	status = MS::kFailure;

	// Find all skin clusters in the scene
	MItDependencyNodes itDep(MFn::kSkinClusterFilter, &status);
	if (status != MS::kSuccess) return MMatrix::identity;

	MString jointName = jointPath.partialPathName();

	while (!itDep.isDone()) {
		MObject skinClusterObj = itDep.thisNode();
		MFnSkinCluster skinCluster(skinClusterObj, &status);
		if (status != MS::kSuccess) {
			itDep.next();
			continue;
		}

		// Get all influence objects for this skin cluster
		MDagPathArray influencePaths;
		int numInfluences = skinCluster.influenceObjects(influencePaths, &status);
		if (status != MS::kSuccess) {
			itDep.next();
			continue;
		}

		// Check if our joint is an influence
		for (int i = 0; i < numInfluences; ++i) {
			if (influencePaths[i].partialPathName() == jointName) {
				unsigned int logicalIndex = skinCluster.indexForInfluenceObject(influencePaths[i], &status);
				if (status != MS::kSuccess) {
					continue;
				}

				MFnDependencyNode skinClusterDepNode(skinClusterObj);
				MPlug bindPreMatrixPlug = skinClusterDepNode.findPlug("bindPreMatrix", &status);
				if (status != MS::kSuccess) {
					continue;
				}

				MPlug matrixPlug = bindPreMatrixPlug.elementByLogicalIndex(logicalIndex, &status);
				if (status != MS::kSuccess) {
					continue;
				}

				MObject matrixData;
				status = matrixPlug.getValue(matrixData);
				if (status != MS::kSuccess) {
					continue;
				}

				MFnMatrixData matrixDataFn(matrixData);
				MMatrix bindPreMatrix = matrixDataFn.matrix(&status);
				if (status == MS::kSuccess) {
					return bindPreMatrix;
				}
			}
		}

		itDep.next();
	}

	// Joint not found in any skin cluster
	return MMatrix::identity;
}
