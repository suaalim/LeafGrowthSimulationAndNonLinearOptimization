#include "SceneNode.h"
#include "Geometry.h"
#include "Log.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"
#include <imgui.h>
#include <memory>
#include <glm/glm.hpp>
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <cstdlib>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/spline.hpp>
#include <map>
#include <numeric>
#include <cmath>
#include <tuple>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include <glm/gtx/rotate_vector.hpp>
#include <functional>
#include "toml.hpp"
#include <unordered_set>
#include "GLDebug.h"
#include <queue>
#include <set>
#include <algorithm>

// helper function to print the matrices for debugging purposes
void printMat4(const glm::mat4& mat) {
	for (int i = 0; i < 4; i++) {
		std::cout << "| ";
		for (int j = 0; j < 4; j++) {
			std::cout << mat[j][i] << "\t";
		}
		std::cout << "|\n";
	}
	std::cout << std::endl;
}

glm::mat4 calculateAnimationMatrix(ContourBinding c1) {
	return (c1.blending) * c1.childNode->marginTransformation + (1 - (c1.blending)) * c1.parentNode->marginTransformation*glm::toMat4(c1.childNode->localRotation);
	// without applying child's localRotation, you are blending the local coordinate frame too
}

glm::mat4 calculateAnimationMatrixForNewPoint(float blending, SceneNode* branch) {
	return  blending * branch->marginTransformation + (1 - blending) * branch->parent->marginTransformation * glm::toMat4(branch->localRotation);
}

glm::vec3 calculateClosestPoint(ContourBinding c1) {
	return (c1.t) * c1.childNode->globalTransformation[3] + (1 - c1.t) * c1.parentNode->globalTransformation[3];
}

glm::vec3 calculateClosestPointForNewPoint(float t, SceneNode* branch) {
	return t * branch->globalTransformation[3] + (1 - (t)) * branch->parent->globalTransformation[3];
}

float calculateT(glm::vec3 P, glm::vec3 Q, glm::vec3 R) {
	return dot(Q - P, R - P) / dot(Q - P, Q - P);
}

float calculateBlending(glm::vec3 P, glm::vec3 Q, glm::vec3 R) {
	return dot(Q - P, R - P) / dot(Q - P, Q - P);
}

glm::vec3 intersectionPoint(glm::vec3 P, glm::vec3 Q, glm::vec3 R) {
	float t = calculateT(P, Q, R);
	if (t <= 0) return P;
	else if (t >= 1) return Q;
	else {
		return P + t * (Q - P);
	}
}

// Scene Graph Structure
SceneNode::SceneNode() : localTranslation(1.0f), localRotation(1.0f, 0.0f, 0.0f, 0.0f), localScaling(1.0f), parent(nullptr) {}

// function to add children to current node
void SceneNode::addChild(SceneNode* child) {
	child->parent = this;
	children.push_back(child);
}

// to remove a child when merging branch
void SceneNode::removeChild(SceneNode* childToRemove) {
	for (size_t i = 0; i < children.size(); i++) {
		if (children[i] == childToRemove) {
			for (size_t j = i; j < children.size() - 1; j++) {
				children[j] = children[j + 1];
			}
			children.pop_back();
			childToRemove->parent = nullptr;
			break;
		}
	}
}

// new branch parameter
void SceneNode::readNewBranchParameter() {
	NewBranchConfig& config = NewBranchConfig::instance();

	newBranchS = config.getNewBranchS();
	newBranchExpansionFactor = config.getNewBranchExpansionFactor();
	newBranchGrowthFactor = config.getNewBranchGrowthFactor();
}

// simulation parameter
void SceneNode::readSimulationParameter() {
	SimulationConfig& config = SimulationConfig::instance();

	subdivisionThreshold = config.getSubdivisionThreshold();
	subdivisionDivision = config.getSubdivisionDivision();
	newContourPointThreshold = config.getNewContourPointThreshold();
	newBindingPointThreshold = config.getNewBindingPointThreshold();
	minRebindContourDistance = config.getMinRebindContourDistance();
	maxRebindContourDistance = config.getMaxRebindContourDistance();
	deltaTime = config.getDeltaTime();
	pointsPerSegment = config.getPointsPerSegment();
	bestBindingStep = config.getBestBindingStep();
	subdivideBranch = static_cast<bool>(config.getsubdivideBranch());
	rebindEveryFrame = static_cast<bool>(config.getRebindEveryFrame());
	perpendicularBranch = static_cast<bool>(config.getPerpendicularBranch());
	blend = static_cast<bool>(config.getBlend());
	indentationWindow = config.getIndentationWindow();
	newBranchDistance = config.getNewBranchDistance();
	newBranchExclusion = config.getNewBranchExclusion();
	petiole = static_cast<bool>(config.getPetiole());
	petiole = static_cast<bool>(config.getPetiole());
	symmetricBranch = static_cast<bool>(config.getSymmetricBranch());
	perpendicularBinding = static_cast<bool>(config.getPerpendicularBinding());
	growthReductionThreshold = config.getGrowthReductionThreshold();
	growthReductionStep = config.getGrowthReductionStep();
	growthReductionFactor = config.getGrowthReductionFactor();
	tipToTipBlending = config.getTipToTipBlending();
}

void SceneNode::readPetioleParameter() {
	PetioleConfig& config = PetioleConfig::instance();

	branchingNodePetioleS = config.getBranchingNodeS();
	branchingNodePetioleExpansion = config.getBranchingNodeExpansion();
	branchingNodePetioleGrowth = config.getBranchingNodeGrowth();
	newBranchPetioleEndS = config.getNewBranchPetioleEndS();
	newBranchPetioleEndExpansion = config.getNewBranchPetioleEndExpansion();
	newBranchPetioleEndGrowth = config.getNewBranchPetioleEndGrowth();
	newBranchPetioleAfterS = config.getNewBranchPetioleAfterS();
	newBranchPetioleAfterExpansion = config.getNewBranchPetioleAfterExpansion();
	newBranchPetioleAfterGrowth = config.getNewBranchPetioleAfterGrowth();
	mainAxisPetioleEndS = config.getMainAxisPetioleEndS();
	mainAxisPetioleEndExpansion = config.getMainAxisPetioleEndExpansion();
	mainAxisPetioleEndGrowth = config.getMainAxisPetioleEndGrowth();
	mainAxisPetioleAfterS = config.getMainAxisPetioleAfterS();
	mainAxisPetioleAfterExpansion = config.getMainAxisPetioleAfterExpansion();
	mainAxisPetioleAfterGrowth = config.getMainAxisPetioleAfterGrowth();
	newBranchDivision1 = config.getNewBranchDivision1();
	newBranchDivision2 = config.getNewBranchDivision2();
	newBranchDivision3 = config.getNewBranchDivision3();
	mainAxisDivision1 = config.getMainAxisDivision1();
	mainAxisDivision2 = config.getMainAxisDivision2();
	mainAxisDivision3 = config.getMainAxisDivision3();
}

// branching structure parameter
std::vector<std::vector<int>> SceneNode::buildChildrenList(
	const std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>>& edges
) {
	// maximum node index
	int maxIndex = 0;
	for (const auto& [parent, child, rot, scale, trans, scaleF, rotationD, rotationA, expansionF, growthF, branchPos, axis, branch] : edges) {
		maxIndex = std::max({ maxIndex, parent, child });
	}

	std::vector<std::vector<int>> childrenList(maxIndex + 1);

	for (const auto& [parent, child, rot, scale, trans, scaleF, rotationD, rotationA, expansionF, growthF, branchPos, axis, branch] : edges) {
		childrenList[parent].push_back(child);

	}

	return childrenList;
}

// create branching structure from the parameter files
SceneNode* SceneNode::createBranchingStructure(
	int nodeIndex, std::vector<std::vector<int>> parentChildPairs, std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> transformations) {
	// create node
	SceneNode* node = new SceneNode();
	node->localTranslation = glm::mat4(1.0f);
	node->localRotation = glm::quat(1.0f, 0.f, 0.f, 0.f);
	node->localScaling = glm::mat4(1.0f);
	node->S = 0.0f;
	node->animationDirection = 0;
	node->rotationAngle = 0.f;
	node->expansionFactor = 3.f;
	node->growthFactor = 5.f;
	node->positionOnBranch = 0.f;
	node->axisID = 0;
	node->branchID = 0;
	node->level = 0;
	node->rebindContourDistance = 0.005;

	// Loop over children of this node
	for (int childIndex : parentChildPairs[nodeIndex]) {
		// Find the transformation for edge (nodeIndex -> childIndex)
		auto it = std::find_if(
			transformations.begin(),
			transformations.end(),
			[nodeIndex, childIndex](const auto& t) {
				return std::get<0>(t) == nodeIndex && std::get<1>(t) == childIndex;
			}
		);

		if (it == transformations.end()) {
			std::cerr << "Missing transformation from " << nodeIndex << " to " << childIndex << "\n";
			continue;
		}

		// Recursively create the child SceneNode
		SceneNode* childNode = createBranchingStructure(childIndex, parentChildPairs, transformations);
		if (!childNode) continue;

		// Set child's local transforms from the tuple
		childNode->localRotation = glm::quat_cast(std::get<2>(*it));
		childNode->localScaling = std::get<3>(*it);
		childNode->localTranslation = std::get<4>(*it);
		childNode->S = std::get<5>(*it);
		childNode->animationDirection = std::get<6>(*it);
		childNode->rotationAngle = std::get<7>(*it);
		childNode->expansionFactor = std::get<8>(*it);
		childNode->growthFactor = std::get<9>(*it);
		childNode->positionOnBranch = std::get<10>(*it);
		childNode->axisID = std::get<11>(*it);
		childNode->branchID = std::get<12>(*it);
		node->addChild(childNode);
	}

	return node;
}
// end of building branching structure 

// animation of the branching structure
bool animated = false;
void SceneNode::animate(float deltaTime) {
	glm::vec3 localY = glm::vec3(0.f, 1.f, 0.f);
	glm::vec3 localX = glm::vec3(-1.f, 0.f, 0.f);

	if (this->parent) {
		localY = glm::normalize(glm::vec3(this->globalTransformation[3] - this->parent->globalTransformation[3]));
		localX = glm::vec3(-localY.y, localY.x, 0.f);
	}

	animated = true;
	// stop animation after certain time
	if (animationTime >= animationDuration) {
		return;
	}
	animationTime += deltaTime;

	animationAngle += deltaTime * rotationAngle * animationDirection;
	animateRotation = glm::toQuat(glm::rotate(glm::mat4(1.0f), glm::radians(animationAngle), glm::vec3(0, 0, 1)));

	if (parent == nullptr) animationScaling = (1 + deltaTime * S) * animationScaling;     // S controls growth
	else animationScaling = (1 + deltaTime * (S * 0.5f + parent->S * 0.5f)) * animationScaling;  // average child and parent because this is branch scaling
	animateScaling = glm::scale(glm::mat4(1.0f), glm::vec3(animationScaling, animationScaling, 1.f));

	if (parent == nullptr) expansionAmount = (1 + deltaTime * expansionFactor) * expansionAmount;        // expansionFactor controls expansion
	else expansionAmount = (1 + deltaTime * (expansionFactor + parent->expansionFactor) / 2.0) * expansionAmount;
	expansion = glm::scale(glm::mat4(1.0f), glm::vec3(expansionAmount, 1.f, 1.f));  // horizontal expansion/expansion in the direction of binding

	if (parent == nullptr) growthAmount = (1 + deltaTime * growthFactor) * growthAmount;    // vertical growth (needed when add new branch)
	else growthAmount = (1 + deltaTime * (growthFactor * 0.5f + parent->growthFactor * 0.5f)) * growthAmount;
	growth = glm::scale(glm::mat4(1.0f), glm::vec3(1.f, growthAmount, 1.f));

	for (SceneNode* child : children) {
		child->animate(deltaTime);
	}
}

// compute transformation for all nodes recursively
void SceneNode::updateBranch(const glm::mat4& parentTransform, const glm::mat4& parentTransformAnimation, const glm::mat4& parentRestInverse, const glm::mat4& parentRest) {
	// convert rotation quaternion back to matrix form
	glm::mat4 animateRotationMatrix = glm::toMat4(animateRotation);  // from quaternion to matrix
	glm::mat4 localRotationMatrix = glm::toMat4(localRotation);      // from quaternion to matrix

	// no rotation, removing scaling 
	glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	temp2[0][0] = 1.f;
	temp2[1][1] = 1.f;
	temp2[2][2] = 1.f;

	glm::mat4 temp3 = animateScaling * localScaling * localTranslation;
	temp3[0][0] = 1.0;
	temp3[1][1] = 1.0;
	temp3[2][2] = 1.0;

	marginTransformation = parentTransform * localRotationMatrix * growth * animateScaling * localScaling * localTranslation * expansion;   // transformation for the current node is set here
	globalTransformation = parentTransform * localRotationMatrix * growth * animateScaling * localScaling * localTranslation;

	// we don't want parent's scaling to affect the child because all children will be overgrown
	// scaling gets accumulated and children grow too much
	glm::mat4 temp1 = growth * animateScaling * localScaling * localTranslation;
	temp1[0][0] = 1.0;
	temp1[1][1] = 1.0;
	temp1[2][2] = 1.0;
	// to note:
	// growth * localTranslation: translation gets scaled in the y component -> when it is applied to point (0, y, 0), point is translated vertically by ty and entire result is vertically scaled
	// localTranslation * growth: translation stays the same -> when applied to point (0, y, 0), point's original height y is scaled vertically then is translated
	// other points have a distance vector between the binding and the contour point to be translated, but the tip point does not, so if you scale before translating then it has no affect
	glm::mat4 newParentTransform = parentTransform * localRotationMatrix * temp1;      // parent transformation that the child will inherit
	glm::mat4 newParentTransformAnimation = newParentTransform;

	// global to local rest post matrix
	// need to apply parentRest outside because if not inverse will accumulate (inverse every call)
	restPoseInverse = glm::inverse(localRotationMatrix * temp1) * parentRestInverse;
	// rest pose matrix
	restPose = parentRest * localRotationMatrix * temp1;
	// remember parent transformation
	parentTransformation = newParentTransform;
	parentTransformationAnimation = newParentTransformAnimation;

	for (SceneNode* child : children) {
		// recurse
		child->updateBranch(newParentTransform, newParentTransformAnimation, restPoseInverse, restPose);
	}
}

void SceneNode::saveBranchGeometry(CPU_Geometry& outGeometry) {
	glm::vec3 rootPos = glm::vec3(globalTransformation[3]);
	unsigned int currentIndex = outGeometry.verts.size();
	outGeometry.verts.push_back(rootPos);
	outGeometry.cols.push_back(glm::vec3(0.f, 0.8f, 0.f));

	for (SceneNode* child : children) {
		unsigned int childIndex = outGeometry.verts.size();
		outGeometry.indices.push_back(currentIndex);
		outGeometry.indices.push_back(childIndex);

		// recurse
		child->saveBranchGeometry(outGeometry);
	}
}

// helper function to increment branchID (position along an axis incremented for all descendants of the same axis)
void SceneNode::incrementBranchIDsOnAxis(SceneNode* node, int axisID)
{
	if (node->axisID == axisID) node->branchID++;

	for (SceneNode* child : node->children)
	{
		incrementBranchIDsOnAxis(child, axisID);
	}
}

// creates midNode at splitPos along node->child edge, rewires child
SceneNode* SceneNode::subdivideEdge(SceneNode* node, SceneNode* child,
	const glm::vec3& splitPos, bool bidirectionalGrowth, int zeroInterpolatedValues) {
	glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
	glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);

	float fullDist = glm::length(childPos - parentPos);
	float t = (fullDist > 1e-8f) ? (glm::length(splitPos - parentPos) / fullDist) : 0.0f;

	SceneNode* midNode = new SceneNode();
	midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
	midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(splitPos - parentPos)));
	midNode->localRotation = child->localRotation;

	// midNode needs to get different growth rates if they are part of the petiole
	// even within petiole, the rates are different if they are part of the main axis or secondary axis
	if (zeroInterpolatedValues == 1) { // branching node
		midNode->S = branchingNodePetioleS;
		midNode->expansionFactor = branchingNodePetioleExpansion;
		midNode->growthFactor = branchingNodePetioleGrowth;
	}
	else if (zeroInterpolatedValues == 2) { // end of petiole region
		midNode->S = newBranchPetioleEndS;
		midNode->expansionFactor = newBranchPetioleEndExpansion;
		midNode->growthFactor = newBranchPetioleEndGrowth;
	}
	else if (zeroInterpolatedValues == 3) { // right after petiole
		midNode->S = newBranchPetioleAfterS;
		midNode->expansionFactor = newBranchPetioleAfterExpansion;
		midNode->growthFactor = newBranchPetioleAfterGrowth;
	}
	else if (zeroInterpolatedValues == 4) { // end of petiole region for main axis
		midNode->S = mainAxisPetioleEndS;
		midNode->expansionFactor = mainAxisPetioleEndExpansion;
		midNode->growthFactor = mainAxisPetioleEndGrowth;
	}
	else if (zeroInterpolatedValues == 5) { // right after petiole for main axis
		midNode->S = mainAxisPetioleAfterS;
		midNode->expansionFactor = mainAxisPetioleAfterExpansion;
		midNode->growthFactor = mainAxisPetioleAfterGrowth;
	}
	// for regular subdivision
	else {
		midNode->S = glm::mix(node->S, child->S, t);
		midNode->expansionFactor = glm::mix(node->expansionFactor, child->expansionFactor, t);
		midNode->growthFactor = glm::mix(node->growthFactor, child->growthFactor, t);
	}
	midNode->rotationAngle = child->rotationAngle;
	midNode->animationDirection = child->animationDirection;
	midNode->animationAngle = child->animationAngle;
	midNode->animateRotation = child->animateRotation;
	midNode->animationScaling = 1;
	midNode->animateScaling = glm::mat4(1.0);
	midNode->axisID = child->axisID;
	midNode->branchID = node->branchID + 1;
	midNode->level = child->level;
	midNode->rebindContourDistance = child->rebindContourDistance;
	incrementBranchIDsOnAxis(child, node->axisID);

	child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
	child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(childPos - splitPos)));
	child->localRotation = glm::mat4(1.f);
	child->rotationAngle = 0;
	child->animationDirection = 0;
	child->animationAngle = 0.f;
	child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	child->animationScaling = 1;
	child->animateScaling = glm::mat4(1.0);
	child->expansion = glm::mat4(1.f);
	child->expansionAmount = 1.f;
	child->growth = glm::mat4(1.f);
	child->growthAmount = 1.f;

	midNode->addChild(child);
	midNode->parent = node;

	node->midBranch = true;
	midNode->midBranch = true;
	child->midBranch = true;

	return midNode;
}

// subdivide entire branching structure recursively
DivideBranchResult SceneNode::divideBranch(SceneNode* node, bool bidirectionalGrowth) {
	std::vector<SceneNode*> originalChildren = node->children;
	std::vector<SceneNode*> updatedChildren;
	std::vector<DivisionResult> localResults;
	bool dividedLocal = false;

	for (SceneNode* child : originalChildren) {
		glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
		float distance = glm::length(childPos - parentPos);

		if (distance > subdivisionThreshold) {
			dividedLocal = true;
			glm::vec3 splitPos = glm::mix(parentPos, childPos, 1.0f / subdivisionDivision);
			SceneNode* midNode = subdivideEdge(node, child, splitPos, bidirectionalGrowth, 0);
			updatedChildren.push_back(midNode);
			localResults.push_back({ node, midNode, child });
		}
		else {
			updatedChildren.push_back(child);
		}
	}
	node->children = updatedChildren;

	for (SceneNode* child : originalChildren) {
		DivideBranchResult childResult = divideBranch(child, bidirectionalGrowth);
		dividedLocal = dividedLocal || childResult.divided;
		localResults.insert(localResults.end(), childResult.results.begin(), childResult.results.end());
	}
	divided = dividedLocal;
	return { dividedLocal, false, localResults };
}

// petiole region -> instead of a recursive function, just divide the node-child branch directly
DivideBranchResult SceneNode::divideSingleBranch(SceneNode* node, SceneNode* child, bool bidirectionalGrowth, float region, int rates) {
	glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
	glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
	glm::vec3 splitPos = glm::mix(parentPos, childPos, 1.0f / region);

	SceneNode* midNode = subdivideEdge(node, child, splitPos, bidirectionalGrowth, rates);

	auto& childList = node->children;
	auto it = std::find(childList.begin(), childList.end(), child);
	if (it != childList.end()) *it = midNode;
	else childList.push_back(midNode);

	node->divided = true;
	return { true, true, { { node, midNode, child } } };
}

// recalculate previous frame transformation (inverse) when using blending
void recalculatePrevFrameInverseBlend(std::vector<ContourBinding>& bindings) {
	std::vector<glm::mat4> transformations;
	for (int i = 0; i < bindings.size(); i++)
	{
		transformations.push_back(calculateAnimationMatrix(bindings[i]));
	}
	for (int i = 0; i < bindings.size(); i++) {
		if (bindings[i].blendTip || bindings[i].blend) {
			bindings[i].previousAnimateInverse = glm::inverse(transformations[i]);
		}
	}
}

// to rebind the contour after subdividing branching structure (either for petiole/regular subdivision/adding new axis)
void SceneNode::rebindContourWithBrokenBranch(SceneNode* node, bool& petiole, std::vector<DivisionResult>& divisionResults, int& i, std::vector<ContourBinding>& bindings) {
	i = 0;
	for (ContourBinding& binding : bindings) {
		bool found = false;
		DivisionResult* matchedResult;
		for (DivisionResult& result : divisionResults) {
			if (binding.parentNode == result.node && binding.childNode == result.child) {
				found = true;
				matchedResult = &result;
				break;
			}
		}
		if (found) {
			double length1 = glm::length(glm::vec3(binding.childNode->globalTransformation[3] - binding.parentNode->globalTransformation[3]));
			double length2 = glm::length(glm::vec3(matchedResult->midNode->globalTransformation[3] - binding.parentNode->globalTransformation[3]));
			double percentage = length2 / length1;    // possibility of floating point error
			if (binding.t < percentage) {
				binding.parentNode = matchedResult->node;
				binding.childNode = matchedResult->midNode;
				binding.t = binding.t/percentage;
				binding.blending = binding.t;
				binding.closestPoint = calculateClosestPoint(binding);
				binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
				if (petiole) binding.petioleRegion = binding.childNode->axisID;  // petiole region
			}
			else if (binding.t > percentage) {
				binding.parentNode = matchedResult->midNode;
				binding.childNode = matchedResult->child;
				binding.t = (binding.t-percentage)/(1.0-percentage);
				binding.blending = binding.t;
				binding.closestPoint = calculateClosestPoint(binding);
				binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			}
			else { // point bound exactly at split point
				binding.parentNode = matchedResult->node;
				binding.childNode = matchedResult->midNode;
				binding.t = 1.f;
				binding.blending = binding.t;
				binding.closestPoint = calculateClosestPoint(binding);
				binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
				if (petiole) binding.petioleRegion = binding.childNode->axisID;  // petiole region
			}
		}
		else {
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
		}
		i++;
	}
	recalculatePrevFrameInverseBlend(bindings);
}

// helper function to decrement branchID for all descendants on the same axis
void SceneNode::decrementBranchIDsOnAxis(SceneNode* node, int axisID)
{
	if (node->axisID == axisID) node->branchID--;

	for (SceneNode* child : node->children)
	{
		incrementBranchIDsOnAxis(child, axisID);
	}
}

// merge branch
bool SceneNode::mergeBranch(SceneNode* node, SceneNode* nodeToRemove, SceneNode* parentToMerge, SceneNode* childToMerge) {
	std::vector<SceneNode*> originalChildren = node->children;
	merged = false;

	if (node == nodeToRemove) {
		glm::vec3 parentPos = glm::vec3(parentToMerge->globalTransformation[3]);

		for (SceneNode* child : originalChildren) {
			glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);

			if (child == childToMerge) {
				//child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
				child->localTranslation = node->localTranslation;
				child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(parentPos - childPos)));
				child->localRotation = node->localRotation;
				child->rotationAngle = node->rotationAngle;
				child->animationDirection = node->animationDirection;
				child->animationAngle = 0.f;
				child->animationScaling = 1;
				child->expansionAmount = 1.f;
				child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
				child->animateScaling = glm::mat4(1.0);
				child->expansion = glm::mat4(1.f);
				//child->growth = glm::mat4(1.f);

				// S, expansionFactor, growthFactor remains unchanged if acropetal or basipetal growth
				if (parentToMerge->S == 0.f && childToMerge->S == 0.f) child->S = nodeToRemove->S;
				if (parentToMerge->expansionFactor == 0.f && childToMerge->expansionFactor == 0.f) child->expansionFactor = nodeToRemove->expansionFactor;
				if (parentToMerge->growthFactor == 0.f && childToMerge->growthFactor == 0.f) child->growthFactor = nodeToRemove->growthFactor;

				decrementBranchIDsOnAxis(child, parentToMerge->axisID);
				child->toMerge = true;
				parentToMerge->toMerge = true;
			}

			child->parent = parentToMerge;
			parentToMerge->addChild(child);
		}

		if (node->parent) {
			node->parent->removeChild(node);
		}

		node->children.clear();
		node->parent = nullptr;

		merged = true;
		return true;
	}

	for (SceneNode* child : originalChildren) {
		merged = merged || mergeBranch(child, nodeToRemove, parentToMerge, childToMerge);
	}

	return merged;
}

// to find the node pair where the midNode was merged
SceneNode* findMergeBranch(SceneNode* node) {
	if (!node) return nullptr;

	for (SceneNode* child : node->children) {
		if (node->toMerge && child->toMerge) {
			return child;
		}

		SceneNode* result = findMergeBranch(child);
		if (result) return result;
	}

	return nullptr;
}

// check if a point belongs on a line segment
bool isPointOnSegment(const glm::vec3& A, const glm::vec3& B, const glm::vec3& P) {
	glm::vec3 AB = B - A;
	glm::vec3 AP = P - A;
	float len = glm::length(AB);
	if (len < 1e-8f) return false; // degenerate segment

	float distanceFromLine = glm::length(glm::cross(AB, AP)) / len;
	if (distanceFromLine > 1e-5f) {
		return false; // not collinear
	}

	float dot = glm::dot(AP, AB);
	if (dot < 0.0f) return false;       // before A
	float lenSq = len * len;
	if (dot > lenSq) return false;      // after B

	return true;
}

// to rebind contour after merging a node in the branching structure
void SceneNode::rebindContourWithMergedBranch(SceneNode* node, std::vector<ContourBinding>& bindings) {
	SceneNode* childBranchNode = findMergeBranch(node);
	SceneNode* parentBranchNode = childBranchNode->parent;

	for (ContourBinding& binding : bindings) {      // if parentNode is the same, we need to make sure that it is in the right branch (there can be multiple branches with the same parent)
		if (binding.childNode == childBranchNode || (binding.parentNode == parentBranchNode && isPointOnSegment(parentBranchNode->globalTransformation[3], childBranchNode->globalTransformation[3], binding.closestPoint))) {
			binding.parentNode = parentBranchNode;
			binding.childNode = childBranchNode;
			binding.t = (glm::length(binding.closestPoint - glm::vec3(parentBranchNode->globalTransformation[3])) / glm::length(childBranchNode->globalTransformation[3] - parentBranchNode->globalTransformation[3]));
			binding.blending = (glm::length(binding.closestPoint - glm::vec3(parentBranchNode->globalTransformation[3])) / glm::length(childBranchNode->globalTransformation[3] - parentBranchNode->globalTransformation[3]));
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			binding.closestPoint = calculateClosestPoint(binding);
			// guarantee t > 0
			if (binding.t == 0 && parentBranchNode->parent != NULL) {
				binding.parentNode = parentBranchNode->parent;
				binding.childNode = parentBranchNode;
				binding.blending = 1;
				binding.t = 1;
				binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
				binding.closestPoint = calculateClosestPoint(binding);
			}
		}
		// points binded to child of merged branch (animation matrix has to be recalculated because the parentNode changed)
		else if (binding.parentNode == childBranchNode) {
			binding.parentNode = childBranchNode;
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			binding.closestPoint = calculateClosestPoint(binding);
			// guarantee t > 0
			if (binding.t == 0 && parentBranchNode->parent != NULL) {
				binding.parentNode = parentBranchNode;
				binding.childNode = childBranchNode;
				binding.blending = 1;
				binding.t = 1;
				binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
				binding.closestPoint = calculateClosestPoint(binding);
			}
		}
	}
}

// finds the contour point symmetric about the y-axis of the one that was clicked in handleAddBranchClick function
// mirrorZ=false mirrors only x (x,y,z)->(-x,y,z);
// mirrorZ=true mirrors x and z (x,y,z)->(-x,y,-z), a true reflection about the y-axis line
// used in dynamically adding new branches
ContourBinding* SceneNode::findSymmetricBinding(std::vector<ContourBinding>& bindings,
	const glm::vec3& point,
	float maxSearchDistance,
	bool mirrorZ)
{
	glm::vec3 mirrored = mirrorZ
		? glm::vec3(-point.x, point.y, -point.z)
		: glm::vec3(-point.x, point.y, point.z);

	ContourBinding* best = nullptr;
	float bestDist = std::numeric_limits<float>::max();

	for (auto& b : bindings) {
		float d = glm::length(b.contourPoint - mirrored);
		if (d < bestDist) {
			bestDist = d;
			best = &b;
		}
	}

	if (maxSearchDistance > 0.0f && bestDist > maxSearchDistance)
		return nullptr; 

	return best;
}

// automatically finding contour points that are within distance (specified in the parameter file) to add new axis
// used in dynamically adding new axis
// within distance means if the current axis is long enough but the contour point is far enough from the tip node
ContourBinding* SceneNode::findBranchAdditionPoints(SceneNode* root, std::vector<ContourBinding>& bindings, float distanceThreshold, float exclusionDistance)
{
	if (bindings.empty()) return nullptr;

	std::vector<float> cumulativeDist(bindings.size(), 0.0f);
	for (size_t k = 1; k < bindings.size(); k++) {
		cumulativeDist[k] = cumulativeDist[k - 1] +
			glm::length(bindings[k].contourPoint - bindings[k - 1].contourPoint);
	}

	ContourBinding bestBinding;
	size_t startIdx = 0;
	float accumulatedDistance = 0.0f;
	glm::vec3 skipThreshold(-0.027f, 0.127f, 4.9f);
	for (size_t i = startIdx + 1; i < bindings.size(); i++) {
		glm::vec3 prevPoint = bindings[i - 1].contourPoint;
		glm::vec3 currPoint = bindings[i].contourPoint;
		accumulatedDistance += glm::length(currPoint - prevPoint);

		if (accumulatedDistance > distanceThreshold) {
			// add an axis only if it is far enough from the root
			if (currPoint.y < skipThreshold.y) {
				continue;
			}
			float minMarkedDistance = std::numeric_limits<float>::max();
			for (int j = 0; j < bindings.size(); j++) {
				bool isLeaf = (bindings[j].leafNodeMarker != -1);
				bool isBranch = (bindings[j].branchingNodeMarker != -1);
				if (isLeaf || isBranch) {
					// arc-length distance along the contour, not straight-line
					float d = std::abs(cumulativeDist[j] - cumulativeDist[i]);
					minMarkedDistance = std::min(minMarkedDistance, d);
				}
			}
			if (minMarkedDistance > exclusionDistance && bindings[i].leafNodeMarker == -1 && bindings[i].branchingNodeMarker == -1) {
				startIdx = i;
				accumulatedDistance = 0.0f;
				return &bindings[i];
			}
		}
	}
	return nullptr;
}

// same as below, but add new axis 90 degrees
ContourBinding SceneNode::findBestBindingPerpendicular(SceneNode* root, const glm::vec3& contourPoint) {
	ContourBinding bestBinding;
	float minDistance = std::numeric_limits<float>::max();

	std::function<void(SceneNode*)> dfs = [&](SceneNode* node) {
		for (SceneNode* child : node->children) {
			glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
			glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
			glm::vec3 dir = childPos - parentPos;

			float segLenSq = glm::dot(dir, dir);
			float t = (segLenSq > 1e-8f)
				? glm::clamp(glm::dot(contourPoint - parentPos, dir) / segLenSq, 0.0f, 1.0f)
				: 0.0f;

			glm::vec3 projected = parentPos + t * dir;
			float d = glm::length(contourPoint - projected);

			if (d < minDistance) {
				minDistance = d;
				bestBinding = { node, child, contourPoint, t, projected, glm::mat4(1.0f), false, t };
			}

			dfs(child);
		}
		};
	dfs(root);
	return bestBinding;
}

// finds the best location on any existing branch to attach a new axis
// visits every parent-child relationship
// so it is not necessarily on the branch that contourPoint is bound to
ContourBinding SceneNode::findBestBinding(SceneNode* root, const glm::vec3& contourPoint) {  // works with subdivided axis
	ContourBinding bestBinding;
	float minTotalDistance = std::numeric_limits<float>::max();

	// just consider main axis for now (one)
	std::function<void(SceneNode*)> dfs = [&](SceneNode* node) {
		for (SceneNode* child : node->children) {
			glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
			glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
			glm::vec3 dir = childPos - parentPos;

			const int steps = bestBindingStep;
			for (int i = 0; i <= steps; i++) {
				float t = static_cast<float>(i) / steps;

				glm::vec3 projected = parentPos + t * dir;
				//glm::vec3 projected = glm::vec3(0.f, 0.5f, 0.f);  // hardcode the projected point for debugging

				// distance from contour point to the test point on the axis
				float d1 = glm::length(contourPoint - projected);

				// accumulated distance from projected point back to root
				float d2 = glm::length(projected - parentPos);
				SceneNode* current = node;
				while (current->parent) {
					glm::vec3 currPos = glm::vec3(current->globalTransformation[3]);
					glm::vec3 parentPos = glm::vec3(current->parent->globalTransformation[3]);
					d2 += glm::length(currPos - parentPos);
					current = current->parent;
				}

				float totalDistance = d1 + (0.5f * d2);
				if (totalDistance < minTotalDistance && projected != glm::vec3(0.f, 0.f, 0.f)) {   // don't want to add a new axis from the root
					minTotalDistance = totalDistance;

					if (t == 1) {     // special case for tip of the axis, to preserve the tip of the main axis
						t = 0.85f;
						projected = parentPos + t * dir;
					}

					bestBinding = {
						node,
						child,
						contourPoint,
						t,
						projected,
						glm::mat4(1.0f),
						false,
						t
					};
				}
			}

			// recurse
			dfs(child);
		}
		};

	dfs(root);
	return bestBinding;
}

// when adding a new axis, if the split point is very close to an existing node, use this function to
// find the existing node
static SceneNode* findExistingNodeNear(SceneNode* current, const glm::vec3& targetPos, float epsilon)
{
	if (!current) return nullptr;

	glm::vec3 currentPos = glm::vec3(current->globalTransformation[3]);
	if (glm::length(currentPos - targetPos) <= epsilon) {
		return current;
	}

	for (SceneNode* c : current->children) {
		if (SceneNode* found = findExistingNodeNear(c, targetPos, epsilon)) {
			return found;
		}
	}
	return nullptr;
}

// subdivide to add new axis
DivideBranchResult SceneNode::splitAtBinding(ContourBinding* pointToBreak) {
	SceneNode* node = pointToBreak->parentNode;
	SceneNode* child = pointToBreak->childNode;

	if (!node || !child) {
		return { false, {} };
	}

	std::vector<DivisionResult> results;

	// if a node already exists at/near the split point, reuse it instead of creating one -> not used 
	//SceneNode* existingNode = findExistingNodeNear(this, pointToBreak->closestPoint, 1e-2f);
	//if (existingNode) {
	//	node->midBranch = true;
	//	existingNode->midBranch = true;
	//	child->midBranch = true;
	//	node->trackOriginalBranch = true;
	//	child->trackOriginalBranch = true;
	//	pointToBreak->parentNode->trackOriginalBranch = true;
	//	pointToBreak->childNode->trackOriginalBranch = true;
	//	existingNode->addBranch = true;

	//	results.push_back({ node, existingNode, child });
	//	return { true, false, results };
	//}

	SceneNode* midNode = subdivideEdge(node, child, pointToBreak->closestPoint,
		/*bidirectionalGrowth=*/false, 0);

	node->trackOriginalBranch = true;
	child->trackOriginalBranch = true;
	pointToBreak->parentNode->trackOriginalBranch = true;
	pointToBreak->childNode->trackOriginalBranch = true;
	midNode->addBranch = true;

	auto& childList = node->children;
	auto it = std::find(childList.begin(), childList.end(), child);
	if (it != childList.end()) *it = midNode;
	else childList.push_back(midNode);

	divided = true;
	results.push_back({ node, midNode, child });
	return { true, false, results };
}

glm::quat SceneNode::accumulateRotationToRoot(SceneNode* node) {
	if (!node) return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	glm::quat accumulated = node->localRotation;
	SceneNode* current = node->parent;

	while (current) {
		accumulated = current->localRotation * accumulated;
		current = current->parent;
	}

	return accumulated;
}

// find the branch that was divided (in splitAtBinding), add a new axis
SceneNode* SceneNode::addNewBranch(SceneNode* node, ContourBinding* contour, int& maxID) {
	if (node->addBranch) {
		// found the midNode that we divided, add a new axis from here
		SceneNode* newNode = new SceneNode();
		// Basic transformation setup
		newNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
		newNode->localScaling = glm::scale(glm::mat4(1.0f),
			glm::vec3(glm::length(contour->contourPoint - glm::vec3(node->globalTransformation[3]))));
		glm::vec3 normalizedDirection = glm::normalize(contour->contourPoint - glm::vec3(node->globalTransformation[3]));
		// glm::mat is column-row major order
		glm::mat4 rotationMatrix = glm::mat4(1.0f);
		rotationMatrix[0][0] = normalizedDirection[1];
		rotationMatrix[0][1] = -normalizedDirection[0];
		rotationMatrix[1][0] = normalizedDirection[0];
		rotationMatrix[1][1] = normalizedDirection[1];
		// need to accumulate all the parents' rotatation matrix until root and take the inverse to apply rotation in local coordinate
		newNode->localRotation = rotationMatrix * glm::toMat4(glm::inverse(accumulateRotationToRoot(node)));
		newNode->positionOnBranch = 1.f;
		newNode->animationDirection = 1.f;

		newNode->axisID = maxID + 1;
		newNode->branchID = 0;
		newNode->level = node->level + 1;
		newNode->rebindContourDistance = node->rebindContourDistance / ((float)newNode->level);

		// for S and growthFactor, since newNode's parameter is inherited from node's, in order for newNode to grow fast enough, it has to be bigger than 1
		// also has to do with the fact that the newly added axis is way shorter compared to the main axis
		if (contour->contourPoint.y >= 1.5f) {
			newNode->S = (newBranchS * newNode->level) / contour->contourPoint.y;   // controls how fast or slow this new node will grow, needs to be extremely big for the new axis to grow fast enough cuz node->S is too small
			newNode->expansionFactor = (newBranchExpansionFactor * newNode->level) / 1.5f;
			newNode->growthFactor = (newBranchGrowthFactor * newNode->level);
		}
		else {
			newNode->S = (newBranchS * newNode->level);   // controls how fast or slow this new node will grow, needs to be extremely big for the new axis to grow fast enough cuz node->S is too small
			newNode->expansionFactor = (newBranchExpansionFactor * newNode->level);
			newNode->growthFactor = (newBranchGrowthFactor * newNode->level);
		}

		// Connect to tree
		newNode->parent = node;
		node->addChild(newNode);
		node->addBranch = false;
		// to add new axis
		newNode->addBranch = true;

		return newNode;
	}
	else {
		for (SceneNode* child : node->children) {
			SceneNode* result = addNewBranch(child, contour, maxID);
			if (result) return result;
		}
	}
	return nullptr;
}

// finds neighboring contour points around a given binding c -> used in rebinding to the newly added axis
// detects override if taking contour points from another secondary axis
std::vector<ContourBinding*> SceneNode::getNearbyBindings(ContourBinding* c, std::vector<ContourBinding>& bindings, SceneNode* newNode) {
	float rebindContourDistance = newNode->rebindContourDistance;
	int limit = 3;
	int leftCounter = 0;
	int rightCounter = 0;
	std::vector<ContourBinding*> result;
	int index = -1;
	for (int i = 0; i < bindings.size(); i++) {
		if (bindings[i].contourPoint == c->contourPoint) {
			index = i;
			break;
		}
	}
	// LEFT SIDE
	for (int i = index - 1; i >= 0; i--) {
		// detecting override   
		// child axisID are different where the parent axisID are the same (and make sure bindings[i] isn't bound to main axis)
		// can only rebind if the contour point is bound to the main axis

		// want to override from a non-direct parent axis
		if (bindings[i].bindingAxisID != newNode->parent->axisID) {  // note that newNode is not subdivided yet, so using direct parent works
			if (leftCounter < limit || bindings[i].leafNodeMarker != -1) {
				// allow override, the overriden axis will lose the branching node
			}
			else {
				std::cout << "override left, limit reached: " << i << std::endl;
				overrideIdxLeft = i;
				overrideBranchLeft = bindings[i].childNode;
				break;
			}
		}
		// don't ever override a leaf node
		else if (bindings[i].leafNodeMarker != -1) {
			std::cout << "override left (past leaf node), limit reached: " << i << std::endl;
			overrideIdxLeft = i;
			overrideBranchLeft = bindings[i].childNode;
			break;
		}
		float dist = glm::distance(bindings[i].contourPoint, c->contourPoint);
		if (dist <= rebindContourDistance) {
			result.push_back(&bindings[i]);
			leftCounter += 1;
		}
		else {
			break;
		}
	}

	std::reverse(result.begin(), result.end());
	result.push_back(c);

	// RIGHT SIDE
	for (int i = index + 1; i < static_cast<int>(bindings.size()); i++) {
		float dist = glm::distance(bindings[i].contourPoint, c->contourPoint);
		if (bindings[i].bindingAxisID != newNode->parent->axisID) {
			// want to override from a non-direct parent axis
			if (rightCounter < limit || bindings[i].leafNodeMarker != -1) {
				// allow override, the overriden axis will lose the branching node
			}
			else {
				std::cout << "override right, limit reached: " << i << std::endl;
				overrideIdxRight = i;
				overrideBranchRight = bindings[i].childNode;
				break;
			}
		}
		// don't ever override a leaf node
		else if (bindings[i].leafNodeMarker != -1) {
			std::cout << "override right (past leaf node), limit reached: " << i << std::endl;
			overrideIdxRight = i;
			overrideBranchRight = bindings[i].childNode;
			break;
		}
		if (dist <= rebindContourDistance) {
			result.push_back(&bindings[i]);
			rightCounter += 1;
		}
		else {
			break;
		}
	}
	return result;
}

// finding the root/branching node of any axis (root if main axis, branching node if secondary axis)
SceneNode* findBranchRoot(SceneNode* node) {
	int branchAxisID = node->axisID;
	SceneNode* current = node;
	//while (current->parent != nullptr && current->parent->axisID == branchAxisID) {
	while (current->parent != nullptr && current->parent->children.size() == 1) {
		current = current->parent;
	}
	//// in case override branch is the branching node itself
	//// if you start the while loop with a branching node, it will return the grandparent
	//if (node->children.size() > 1) current = node;
	return current;
}

// function to add contour point if there is override -> add to the branch that got overriden/taken over
void SceneNode::addContourOverride(std::vector<ContourBinding>& bindings, SceneNode* newNode) {
	bool isLeft;
	SceneNode* overrideBranch;
	int overrideIdx;

	if (overrideIdxLeft != -1) {
		isLeft = true;
		overrideBranch = overrideBranchLeft;
		overrideIdx = overrideIdxLeft;
		overrideIdxLeft = -1;
	}
	else if (overrideIdxRight != -1) {
		isLeft = false;
		overrideBranch = overrideBranchRight;
		overrideIdx = overrideIdxRight;
		overrideIdxRight = -1;
	}
	else {
		return; // nothing to do
	}

	ContourBinding newBinding;
	// need the if else statement to set branching node, not its parent
	if (findBranchRoot(overrideBranch)->parent != nullptr) newBinding.childNode = findBranchRoot(overrideBranch)->parent;
	else newBinding.childNode = overrideBranch;
	newBinding.parentNode = newBinding.childNode->parent;
	newBinding.t = 1.f;
	newBinding.blending = 1.f;
	newBinding.closestPoint = calculateClosestPoint(newBinding);

	// left blends toward the next point, right blends toward the previous one
	int neighborIdx = isLeft ? overrideIdx + 1 : overrideIdx - 1;
	newBinding.contourPoint = glm::mix(bindings[overrideIdx].contourPoint,
		bindings[neighborIdx].contourPoint, 0.5f);

	newBinding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(newBinding));
	newBinding.newBranchBinding = true;
	newBinding.uniqueKey = ++contourKey;
	newBinding.bindingAxisID = overrideBranch->axisID;
	newBinding.branchingNodeMarker = overrideBranch->axisID;
	newBinding.leafNodeMarker = -1;

	int insertPos = isLeft ? overrideIdx + 1 : overrideIdx;
	bindings.insert(bindings.begin() + insertPos, newBinding);
}

// rebind all the contour points to the new axis from getNearbyBindings
void SceneNode::rebindToNewBranch(SceneNode* newNode, SceneNode* root, ContourBinding* contour, std::vector<ContourBinding>& bindings) {
	std::vector<ContourBinding*> toRebind = getNearbyBindings(contour, bindings, newNode);

	int index = -1;
	for (int i = 0; i < toRebind.size(); i++) {
		if (toRebind[i]->contourPoint == contour->contourPoint) {
			index = i;   // index is tip point of the new axis
			break;
		}
	}

	if (index != -1) {
		float leftContourDistance = glm::length(toRebind[0]->contourPoint - toRebind[index]->contourPoint);
		for (int i = 0; i < index; i++) {
			if (i == 0) { // first point is bound to a branching node -> different parent & child pair
				toRebind[i]->parentNode = newNode->parent->parent;
				toRebind[i]->childNode = newNode->parent;
				toRebind[i]->blending = 1.f;
				toRebind[i]->t = 1.f;
				toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
				toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
				toRebind[i]->newBranchBinding = true;
				toRebind[i]->branchingNodeMarker = newNode->parent->axisID;
				toRebind[i]->leafNodeMarker = -1;
				toRebind[i]->bindingAxisID = newNode->axisID;
				continue;
			}
			toRebind[i]->parentNode = newNode->parent;
			toRebind[i]->childNode = newNode;
			float leftCurrentDistance = glm::length(toRebind[i]->contourPoint - toRebind[index]->contourPoint);
			toRebind[i]->blending = glm::clamp(1 - ((float)leftCurrentDistance / leftContourDistance), 0.f, 1.f);
			toRebind[i]->t = glm::clamp(1 - ((float)leftCurrentDistance / leftContourDistance), 0.f, 1.f);
			//toRebind[i]->blending = glm::clamp(((float)i / index), 0.f, 1.f);
			//toRebind[i]->t = glm::clamp(((float)i / index), 0.f, 1.f);
			toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
			toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
			toRebind[i]->newBranchBinding = true;
			toRebind[i]->bindingAxisID = newNode->axisID;
			toRebind[i]->branchingNodeMarker = -1;
			toRebind[i]->leafNodeMarker = -1;
		}

		float rightContourDistance = glm::length(toRebind[toRebind.size() - 1]->contourPoint - toRebind[index]->contourPoint);

		for (int i = index + 1; i < toRebind.size(); i++) {
			if (i == toRebind.size() - 1) { // last point is bound to a branching node -> different parent & child pair
				toRebind[i]->parentNode = newNode->parent->parent;
				toRebind[i]->childNode = newNode->parent;
				toRebind[i]->blending = 1.f;
				toRebind[i]->t = 1.f;
				toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
				toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
				toRebind[i]->newBranchBinding = true;
				toRebind[i]->branchingNodeMarker = newNode->parent->axisID;
				toRebind[i]->bindingAxisID = newNode->axisID;
				toRebind[i]->leafNodeMarker = -1;
				continue;
			}
			toRebind[i]->parentNode = newNode->parent;
			toRebind[i]->childNode = newNode;
			float rightCurrentDistance = glm::length(toRebind[i]->contourPoint - toRebind[index]->contourPoint);
			toRebind[i]->blending = glm::clamp(1 - ((float)rightCurrentDistance / rightContourDistance), 0.f, 1.f);
			toRebind[i]->t = glm::clamp(1 - ((float)rightCurrentDistance / rightContourDistance), 0.f, 1.f);
			//toRebind[i]->blending = glm::clamp(1 - ((float)(i - index) / (toRebind.size() - index - 1)), 0.f, 1.f);
			//toRebind[i]->t = glm::clamp(1 - ((float)(i - index) / (toRebind.size() - index - 1)), 0.f, 1.f);
			toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
			toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
			toRebind[i]->newBranchBinding = true;
			toRebind[i]->bindingAxisID = newNode->axisID;
			toRebind[i]->branchingNodeMarker = -1;
			toRebind[i]->leafNodeMarker = -1;
		}
		toRebind[index]->parentNode = newNode->parent;
		toRebind[index]->childNode = newNode;
		toRebind[index]->blending = 1;
		toRebind[index]->t = 1;
		toRebind[index]->closestPoint = calculateClosestPoint(*toRebind[index]);
		toRebind[index]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[index]));
		toRebind[index]->newBranchBinding = true;
		toRebind[index]->bindingAxisID = newNode->axisID;
		toRebind[index]->leafNodeMarker = newNode->axisID;
		toRebind[index]->branchingNodeMarker = -1;
	}
}

// ----- to reorganize children (leftmost to rightmost)
glm::vec3 safeNormalize(const glm::vec3& v, const glm::vec3& fallback = glm::vec3(0.f, 0.f, 1.f))
{
	float len = glm::length(v);
	if (len < 1e-5f) return fallback;
	return v / len;
}

glm::vec3 getDirection(SceneNode* a, SceneNode* b)
{
	glm::vec3 diff = glm::vec3(b->globalTransformation[3]) -
		glm::vec3(a->globalTransformation[3]);
	return safeNormalize(diff);
}

void buildFrame(const glm::vec3& forward, glm::vec3& right, glm::vec3& up)
{
	glm::vec3 worldUp(0.f, 1.f, 0.f);
	glm::vec3 alt(0.f, 0.f, 1.f);

	glm::vec3 rightRaw = glm::cross(worldUp, forward);
	if (glm::length(rightRaw) < 1e-5f)
		rightRaw = glm::cross(alt, forward); // try a second axis
	if (glm::length(rightRaw) < 1e-5f)
		rightRaw = glm::vec3(1.f, 0.f, 0.f); // absolute fallback

	right = glm::normalize(rightRaw);
	up = glm::normalize(glm::cross(forward, right));
}

float computeChildAngle(SceneNode* parent, SceneNode* child)
{
	glm::vec3 parentForward = parent->parent
		? getDirection(parent->parent, parent)
		: glm::vec3(0.f, 0.f, 1.f);

	glm::vec3 right, up;
	buildFrame(parentForward, right, up);

	glm::vec3 childDir = getDirection(parent, child);

	float dotR = glm::dot(childDir, right);
	float dotU = glm::dot(childDir, up);
	float angle = atan2(dotR, dotU);

	return angle;
}

void SceneNode::reorganizeChildrenLeft(SceneNode* node)
{
	if (!node) return;

	// skip sorting if only one child
	if (node->children.size() > 1)
	{
		std::sort(node->children.begin(), node->children.end(),
			[node](SceneNode* a, SceneNode* b)
			{
				float angleA = computeChildAngle(node, a);
				float angleB = computeChildAngle(node, b);
				return angleA > angleB;
			});
	}

	for (SceneNode* child : node->children)
		reorganizeChildrenLeft(child);
}

void SceneNode::reorganizeChildrenRight(SceneNode* node)
{
	if (!node) return;

	// skip sorting if only one child
	if (node->children.size() > 1)
	{
		std::sort(node->children.begin(), node->children.end(),
			[node](SceneNode* a, SceneNode* b)
			{
				float angleA = computeChildAngle(node, a);
				float angleB = computeChildAngle(node, b);
				return angleA < angleB;
			});
	}

	for (SceneNode* child : node->children)
		reorganizeChildrenRight(child);
}
// ------ end of reorganizing children ----------

bool isAncestorOf(SceneNode* ancestor, SceneNode* descendant) {
	SceneNode* current = descendant;
	while (current != nullptr) {
		if (current == ancestor) return true;
		current = current->parent;
	}
	return false;
}

// helper function for global rebinding to find the subdivided segment for each axis
std::pair<SceneNode*, SceneNode*> findSubdividedSegment(glm::vec3 closestPoint, SceneNode* parent, SceneNode* child) {
	// gather subdivided nodes by walking UP from child to parent
	std::vector<SceneNode*> nodes;
	SceneNode* current = child;
	while (current != nullptr) {
		nodes.push_back(current);
		if (current == parent) break;
		current = current->parent; 
	}

	// if we never reached `parent`, the chain is broken/disconnected
	if (nodes.empty() || nodes.back() != parent) {
		return { nullptr, nullptr };
	}

	std::reverse(nodes.begin(), nodes.end()); // now ordered parent -> ... -> child

	// build consecutive segment pairs
	std::vector<std::pair<SceneNode*, SceneNode*>> segments;
	segments.reserve(nodes.size() > 0 ? nodes.size() - 1 : 0);
	for (size_t i = 0; i + 1 < nodes.size(); ++i) {
		segments.emplace_back(nodes[i], nodes[i + 1]);
	}

	// find the segment closestPoint belongs to
	for (auto& branch : segments) {
		glm::vec3 A = glm::vec3(branch.first->globalTransformation[3]);
		glm::vec3 B = glm::vec3(branch.second->globalTransformation[3]);

		if (!isPointOnSegment(A, B, closestPoint)) {
			continue;
		}
		return branch; 
	}

	// no segment matched 
	return { nullptr, nullptr };
}

// rebinding the contour -> contour is sectioned off using branchingNodeMarker/leafNodeMarker
// use this idea to rebind to the correct axis
void SceneNode::calculateRebindingGlobal(std::vector<ContourBinding*>& bindings) {
	int start = 0;
	int regionSize = 0;
	for (int i = 0; i < bindings.size(); i++) {
		if (i == 0) continue;
		regionSize += 1;
		// branching node or leaf node marker detected
		if (bindings[i]->branchingNodeMarker != -1 || bindings[i]->leafNodeMarker != -1) {
			SceneNode* startNode;
			SceneNode* endNode;
			
			// terminal leaflet
			if (bindings[i]->terminalLeaflet) {
				startNode = bindings[i]->childNode;
				endNode = bindings[i]->childNode->children[0];
			}
			else {
				// observation: due to how we bind contour to branch, unless we are starting at the root, we always use the childnode (think about how the branching node marker is bound)
				if (start == 0) startNode = bindings[start]->parentNode;  // cannot use i instead of start because we skip the outer loop for i = 0
				else startNode = bindings[start]->childNode;
				// similar to startNode, for the root node, we use parent node; child node otherwise
				if (i == bindings.size() - 1) endNode = bindings[i]->parentNode;
				else endNode = bindings[i]->childNode;
			}

			// determines which is parent and child node, since return pass flips the order
			SceneNode* hierarchyParent;
			SceneNode* hierarchyChild;
			if (isAncestorOf(startNode, endNode)) {
				hierarchyParent = startNode;
				hierarchyChild = endNode;
			}
			else {
				hierarchyParent = endNode;
				hierarchyChild = startNode;
			}

			// rebind
			for (int j = start; j <= i; j++) {
				if (j == start || j == i) continue; // keep first and last point to root
				bindings[j]->parentNode = startNode;
				bindings[j]->childNode = endNode;
				float t;
				if (perpendicularBinding) t = calculateT(startNode->globalTransformation[3], endNode->globalTransformation[3], bindings[j]->contourPoint);
				else t = 1.0 - ((float)i - (float)j) / ((float)i - (float)start);
				bindings[j]->t = glm::clamp(t, 0.0f, 1.0f);
				//std::cout << i << ", " << j << ", " << start << std::endl;
				bindings[j]->blending = bindings[j]->t;
				bindings[j]->closestPoint = calculateClosestPoint(*bindings[j]);
				// find the subdivided segment on the axis 
				auto segment = findSubdividedSegment(bindings[j]->closestPoint, hierarchyParent, hierarchyChild);
				if (segment.first != nullptr && segment.second != nullptr) {
					bindings[j]->parentNode = segment.first;
					bindings[j]->childNode = segment.second;
					float distance = glm::length(segment.second->globalTransformation[3] - segment.first->globalTransformation[3]);
					float bindingDistance = glm::length(bindings[j]->closestPoint - glm::vec3(segment.first->globalTransformation[3]));
					float t;
					if (perpendicularBinding) t = bindingDistance / distance;
					else t = calculateT(segment.first->globalTransformation[3], segment.second->globalTransformation[3], bindings[j]->contourPoint);
					bindings[j]->t = glm::clamp(t, 0.f, 1.f);
					bindings[j]->blending = bindings[j]->t;
				}
				bindings[j]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*bindings[j]));
			}
			start = i;
			regionSize = 0;
			// terminal leaflet: need to rebind the ith point too 
			if (bindings[i]->terminalLeaflet) {
				bindings[i]->parentNode = endNode->parent;
				bindings[i]->childNode = endNode;
				bindings[i]->t = 1.f;
				bindings[i]->blending = bindings[i]->t;
				bindings[i]->closestPoint = calculateClosestPoint(*bindings[i]);
				bindings[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*bindings[i]));
				bindings[i]->terminalLeaflet = false;
			}
		}
	}
}

// get all leaf nodes -> to be used to generate the control points of the contour 
void SceneNode::getLeafNodes(SceneNode* node, std::vector<SceneNode*>& leaves) {
	if (!node) return;
	if (node->children.empty()) {
		leaves.push_back(node);
	}
	else {
		for (SceneNode* child : node->children) {
			getLeafNodes(child, leaves);
		}
	}
}

// initial control points for contour using root and leaf nodes
std::vector<glm::vec3> SceneNode::generateInitialContourControlPoints(SceneNode* root) {
	std::vector<glm::vec3> controlPoints;

	// root
	glm::vec3 rootPos = glm::vec3(root->globalTransformation[3]);

	glm::vec3 leftOffset = rootPos - glm::vec3(0.025f, 0.0f, 0.0f);
	glm::vec3 rightOffset = rootPos + glm::vec3(0.025f, -0.0f, 0.0f);

	controlPoints.push_back(leftOffset);

	// leaf
	std::vector<SceneNode*> leaves;
	getLeafNodes(root, leaves);

	for (SceneNode* leaf : leaves) {
		glm::vec3 leafPos = leaf->globalTransformation[3];
		glm::vec3 leafParentPos = leaf->parent->globalTransformation[3];
		glm::vec3 dir = glm::normalize(leafPos - leafParentPos);
		glm::vec3 offsetPos = leafPos;
		//glm::vec3 offsetPos = leafPos + glm::vec3(0.f, 0.005f, 0.f);
		controlPoints.push_back(offsetPos);
	}

	controlPoints.push_back(rightOffset);

	return controlPoints;
}

std::vector<glm::vec3> SceneNode::midPoints(std::vector<glm::vec3>& contourPoints) {
	std::vector<glm::vec3> copyOfContour;
	copyOfContour.push_back(contourPoints[0]);
	for (int i = 1; i < contourPoints.size() - 2; i++) {
		copyOfContour.push_back(contourPoints[i]);
		copyOfContour.push_back((contourPoints[i] + contourPoints[i + 1]) / 2.f);
	}
	copyOfContour.push_back(contourPoints[contourPoints.size() - 2]);
	copyOfContour.push_back(contourPoints[contourPoints.size() - 1]);
	return copyOfContour;
}

std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> SceneNode::contourCatmullRomGrouped(std::vector<glm::vec3> controlPoints, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches) {
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContourPoints;
	std::pair<SceneNode*, SceneNode*> segment;

	// catmull rom needs 4 points to interpolate between two middle points
	// create artificial point
	std::vector<glm::vec3> paddedPoints;
	glm::vec3 first = controlPoints[0] + (controlPoints[0] - controlPoints[1]);
	glm::vec3 last = controlPoints.back() + (controlPoints.back() - controlPoints[controlPoints.size() - 2]);

	paddedPoints.push_back(first);
	paddedPoints.insert(paddedPoints.end(), controlPoints.begin(), controlPoints.end());
	paddedPoints.push_back(last);

	int branchCounter = 0;
	int branchIndex = 0;
	// each iteration generates one segment
	for (size_t i = 0; i < paddedPoints.size() - 3; i++) {
		std::vector<glm::vec3> segmentPoints;
		
		for (int j = 0; j < pointsPerSegment; j++) {
			float t = float(j) / (pointsPerSegment);
			glm::vec3 pt = glm::catmullRom(
				paddedPoints[i],
				paddedPoints[i + 1],   // interpolate between [i+1] and [i+2]
				paddedPoints[i + 2],
				paddedPoints[i + 3],
				t
			);
			segmentPoints.push_back(pt);
			segment = { std::get<0>(branches[branchIndex]), std::get<1>(branches[branchIndex]) };

		}
		groupedContourPoints.push_back({ segmentPoints, segment });  // generated segment gets linked to associated branch
		branchCounter++;
		if (branchCounter > 1) {
			branchCounter = 0;
			branchIndex++;
		}

		// last contour point
		if (i == paddedPoints.size() - 4) groupedContourPoints.push_back({ { glm::catmullRom(
			paddedPoints[i],
			paddedPoints[i + 1],
			paddedPoints[i + 2],
			paddedPoints[i + 3],
			1
		) }, segment });
	}

	return groupedContourPoints;
}

// even interpolation along the branching structure for binding -> key for interpolative skinning
std::vector<ContourBinding> SceneNode::bindInterpolatedContourToBranches(std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>>& contourPoints) {
	std::vector<ContourBinding> bindings;
	float t = 0;
	float blending = 0;
	// for each segment
	for (int i = 0; i < contourPoints.size(); i++) {
		// for individual points in the segment
		for (int j = 0; j < contourPoints[i].first.size(); j++) {
			SceneNode* parent = contourPoints[i].second.first; // associated parent node
			SceneNode* child = contourPoints[i].second.second; // associated child node
			glm::vec3 P = parent->globalTransformation[3];
			glm::vec3 Q = child->globalTransformation[3];
			ContourBinding bestBinding;

			// closest point
			if (i % 2 == 1) t = 1 - (j / ((float)contourPoints[i].first.size()));
			else t = (j / ((float)contourPoints[i].first.size()));  
			if (contourPoints[i].first.size() == 1) t = 0;
			glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, child);

			// blending transformation
			if (i % 2 == 1) blending = 1 - (j / ((float)contourPoints[i].first.size()));
			else blending = (j / ((float)contourPoints[i].first.size()));
			if (contourPoints[i].first.size() == 1) blending = 0;
			glm::mat4 previousAnimateInverse = glm::inverse(calculateAnimationMatrixForNewPoint(blending, child));

			glm::vec3 bindingdirection = (contourPoints[i].first[j] - closestPoint) / 2.f;
			int leafNodeMarker = -1;
			if (t == 1) leafNodeMarker = child->axisID;
			bestBinding = { parent, child, contourPoints[i].first[j], t, closestPoint, previousAnimateInverse, false, blending, contourKey, -1, leafNodeMarker, child->axisID };
			bindings.push_back(bestBinding);
			contourKey += 1;
		}
	}
	// set markers for the first and last contour points
	bindings[0].leafNodeMarker = 0;
	bindings[bindings.size() - 1].leafNodeMarker = 0;
	return bindings;
}

// perpendicular binding
std::vector<ContourBinding> SceneNode::bindContourToBranches(
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>>& contourPoints) {

	std::vector<ContourBinding> bindings;

	for (size_t i = 0; i < contourPoints.size(); i++) {
		SceneNode* parent = contourPoints[i].second.first;
		SceneNode* child = contourPoints[i].second.second;
		glm::vec3 P = parent->globalTransformation[3];
		glm::vec3 Q = child->globalTransformation[3];

		for (size_t j = 0; j < contourPoints[i].first.size(); j++) {
			glm::vec3& contourPoint = contourPoints[i].first[j];

			glm::vec3 closest = intersectionPoint(P, Q, contourPoint);
			float t = glm::dot(Q - P, contourPoint - P) / glm::dot(Q - P, Q - P);
			t = glm::clamp(t, 0.0f, 1.0f);
			int leafNodeMarker = -1;
			if (t == 1) leafNodeMarker = child->axisID;
			ContourBinding binding = {
				parent, child, contourPoint, t, closest,
				glm::inverse(calculateAnimationMatrixForNewPoint(t, child)),
				false, t, contourKey, -1, leafNodeMarker, child->axisID
			};
			contourKey += 1;

			bindings.push_back(binding);
		}
	}
	bindings[0].leafNodeMarker = 0;
	bindings[bindings.size() - 1].leafNodeMarker = 0;
	return bindings;
}

// debugging functions --------------------
void SceneNode::printBranches(SceneNode* node) {
	std::cout << glm::vec3(node->globalTransformation[3]) << std::endl;
	std::cout << node->axisID << std::endl;
	for (SceneNode* child : node->children) {
		printBranches(child);
	}
}

void SceneNode::printRates(SceneNode* node) {
	std::cout << "node rates (S, expansion, growth): " << node->S << ", " << node->expansionFactor << ", " << node->growthFactor << std::endl;
}

void SceneNode::printTree(SceneNode* node, SceneNode* lastPrinted, int depth) {
	bool isBranch = node->children.size() > 1;
	bool isLeaf = node->children.size() == 0;

	if (isBranch || isLeaf) {
		std::string indent(depth * 2, ' ');

		if (lastPrinted != nullptr) {
			std::cout << indent << "node: " << glm::vec3(lastPrinted->globalTransformation[3]) << std::endl;
			std::cout << indent << "  child: " << glm::vec3(node->globalTransformation[3]) << std::endl;
			std::cout << std::endl;
		}
		else {
			// this is the root itself being significant, nothing to pair it with yet
			std::cout << indent << "node: " << glm::vec3(node->globalTransformation[3]) << std::endl;
			std::cout << std::endl;
		}

		// node becomes the new "last printed" reference, and depth increases
		for (SceneNode* child : node->children) {
			printTree(child, node, depth + 1);
		}
	}
	else {
		// internal node — skip printing, keep propagating the same lastPrinted reference
		for (SceneNode* child : node->children) {
			printTree(child, lastPrinted, depth);
		}
	}
}
// -----------------------------------------

// extract parent child pair (endpoints of the branches)
void SceneNode::getBranches(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments) {
	for (SceneNode* child : node->children) {
		segments.push_back({ node, child });
		getBranches(child, segments);
	}
}

// label branches hierarchically
void SceneNode::labelBranches(SceneNode* node, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments, int& i) {
	for (SceneNode* child : node->children) {
		segments.push_back({ node, child, i });
		i++;
		labelBranches(child, segments, i);
	}
}

// get maxID
int SceneNode::getMaxID(SceneNode* node)
{
	int maxID = node->axisID;

	for (auto* child : node->children)
	{
		maxID = std::max(maxID, getMaxID(child));
	}

	return maxID;
}

int getNodeDepth(SceneNode* node) {
	int depth = 0;
	while (node->parent) {
		node = node->parent;
		depth++;
	}
	return depth;
}

SceneNode* getDeeperNode(SceneNode* a, SceneNode* b) {
	int depthA = getNodeDepth(a);
	int depthB = getNodeDepth(b);

	return (depthA >= depthB) ? a : b;
}

// add contour points and bind if distance between two contour points exceed a threshold ------------
void SceneNode::pushInterpolatedContourBinding(std::vector<ContourBinding>& out, const ContourBinding& source, const glm::vec3& newPoint,
	float t, float blending, bool newBranchBindingValue, const ContourBinding& a, const ContourBinding& b)
{
	glm::mat4 inverseAnimMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, source.childNode));
	glm::vec3 closestPointMix = glm::mix(a.closestPoint, b.closestPoint, 0.5f);

	out.push_back({
		source.parentNode,
		source.childNode,
		newPoint,
		t,
		closestPointMix,
		inverseAnimMat,
		newBranchBindingValue,
		blending,
		contourKey,
		-1,
		-1,
		source.childNode->axisID
		});
	contourKey += 1;
}

std::vector<ContourBinding> SceneNode::addContourPoints(std::vector<ContourBinding>& bindings) {
	std::vector<ContourBinding> newBindingSet;
	if (bindings.empty()) {
		return newBindingSet;
	}

	for (size_t i = 0; i + 1 < bindings.size(); i++) {
		const ContourBinding& a = bindings[i];
		const ContourBinding& b = bindings[i + 1];

		newBindingSet.push_back(a);

		// arbitrary threshold
		float distance = glm::length(b.contourPoint - a.contourPoint);
		if (distance <= newContourPointThreshold) {
			continue;
		}

		glm::vec3 newPoint = glm::mix(a.contourPoint, b.contourPoint, 0.5f);
		bool sameBranch = (a.childNode == b.childNode);
		bool blendStatesMatch = (a.blend == b.blend); // both blending, or neither blending

		if (sameBranch && blendStatesMatch) {
			// same branch: average both sides' blending/t.
			float blending = (a.blending + b.blending) / 2.f;
			float t = (a.t + b.t) / 2.f;
			pushInterpolatedContourBinding(newBindingSet, a, newPoint, t, blending, a.newBranchBinding, a, b);
		}
		else if (blendStatesMatch) {
			// different branch, both blend states equal: attach to the deeper/younger branch.
			const ContourBinding& neighbor = (getDeeperNode(a.childNode, b.childNode) == a.childNode) ? a : b;
			float blending = neighbor.blending / 2.f;
			float t = neighbor.t / 2.f;
			// NOTE: newBranchBinding always taken from `a` here, regardless of which side
			pushInterpolatedContourBinding(newBindingSet, neighbor, newPoint, t, blending, a.newBranchBinding, a, b);
		}
		else {
			// exactly one side is blending: the non-blending side is the "neighbor" branch,
			// the blending side supplies newBranchBinding.
			const ContourBinding& neighbor = a.blend ? b : a;
			const ContourBinding& other = a.blend ? a : b;
			float blending = neighbor.blending / 2.f;
			float t = neighbor.t / 2.f;
			pushInterpolatedContourBinding(newBindingSet, neighbor, newPoint, t, blending, other.newBranchBinding, a, b);
		}
	}

	newBindingSet.push_back(bindings.back());
	return newBindingSet;
}
// ----------------------------------------------------------

// set contour points to be bound to terminal leaflet's branching node
std::vector<ContourBinding> SceneNode::addContourPointTerminalLeaflet(std::vector<ContourBinding>& bindings, SceneNode* node) {
	std::vector<ContourBinding> newBindingSet;

	glm::vec3 nodePosition = node->globalTransformation[3];

	int closestIndex1 = -1;
	int closestIndex2 = -1;
	float closestDist1 = std::numeric_limits<float>::max();
	float closestDist2 = std::numeric_limits<float>::max();

	for (int i = 0; i < bindings.size(); i++) {
		float distance = glm::length(bindings[i].contourPoint - nodePosition);

		if (distance < closestDist1) {
			closestDist2 = closestDist1;
			closestIndex2 = closestIndex1;

			closestDist1 = distance;
			closestIndex1 = i;
		}
		else if (distance < closestDist2) {
			closestDist2 = distance;
			closestIndex2 = i;
		}
	}
	bindings[closestIndex1].parentNode = node->parent;
	bindings[closestIndex1].childNode = node;
	bindings[closestIndex1].t = 1.f;
	bindings[closestIndex1].blending = 1.f;
	bindings[closestIndex1].branchingNodeMarker = 0;
	bindings[closestIndex1].leafNodeMarker = -1;
	bindings[closestIndex1].closestPoint = node->globalTransformation[3];
	bindings[closestIndex1].previousAnimateInverse = calculateAnimationMatrix(bindings[closestIndex1]);
	bindings[closestIndex2].parentNode = node->parent;
	bindings[closestIndex2].childNode = node;
	bindings[closestIndex2].t = 1.f;
	bindings[closestIndex2].blending = 1.f;
	bindings[closestIndex2].branchingNodeMarker = 0;
	bindings[closestIndex2].leafNodeMarker = -1;
	bindings[closestIndex2].closestPoint = node->globalTransformation[3];
	bindings[closestIndex2].previousAnimateInverse = calculateAnimationMatrix(bindings[closestIndex2]);

	return newBindingSet;
}

// not used
void SceneNode::calculateNormalDirection(std::vector<ContourBinding>& bindings) {
	for (int i = 1; i < bindings.size() - 1; i++) {
		glm::vec3 firstDirection = glm::normalize(bindings[i].contourPoint - bindings[i - 1].contourPoint);
		glm::vec3 secondDirection = glm::normalize(bindings[i + 1].contourPoint - bindings[i].contourPoint);
		glm::vec3 average = glm::vec3((firstDirection.x + secondDirection.x) / 2.f, (firstDirection.y + secondDirection.y) / 2.f, (firstDirection.z + secondDirection.z) / 2.f);   // take average to get tangent vector 
		bindings[i].normalDirection = glm::vec3(-average.y, average.x, average.z);  // rotate by 90 degrees counterclockwise in 2d
	}
}

// animate contour points for growth -> original method, not used anymore 
void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings, float deltaTime) {
	// first, calculate the delta transformation matrices for all points, store in an array
	// then blend the matrices
	// then apply

	// petiole: limit the growth for the first and last few points
	for (int i = 0; i < bindings.size(); i++) {
		//if (i == 0 || i == bindings.size() - 1) continue;
		glm::mat4 animatedPosMat;
		animatedPosMat = calculateAnimationMatrix(bindings[i]);
		//glm::vec3 bindingPosition = bindings[i].closestPoint;   // translating by -bindingPosition moves it back to local space
		
		bindings[i].contourPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].contourPoint, 1.0f);

		//// growth (translation) in the normal direction
		//if (true) {
		//	bindings[i].contourPoint.x += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.x;
		//	bindings[i].contourPoint.y += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.y;
		//}

		// transformation applied in local coordinate frame
		bindings[i].closestPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].closestPoint, 1.0f);
		bindings[i].previousAnimateInverse = glm::inverse(animatedPosMat);
		//binding.closestPoint = calculateClosestPoint(binding);

		// update t value to reflect the non-linear growth
		//bindings[i].t = (glm::length(bindings[i].closestPoint - glm::vec3(bindings[i].parentNode->globalTransformation[3])) / glm::length(bindings[i].childNode->globalTransformation[3] - bindings[i].parentNode->globalTransformation[3]));

	}
}

// type 4 blending: combination of max blending (window blending but from one tip to another tip)
// and tip to tip blending (linear interpolation between tips)
void SceneNode::indentationControlMaxAndTipBlending(std::vector<ContourBinding>& bindings) {
	if (bindings.empty()) return;

	const int n = static_cast<int>(bindings.size());

	for (auto& b : bindings) {
		b.blend = false;
		b.blendRegionBegining = -1;
		b.blendRegionEnd = -1;
		b.blendingWeights.clear();
		b.blendTip = false;
		b.blendRegionBeginingTip = -1;
		b.blendRegionEndTip = -1;
		b.blendingWeightsTip.clear();
	}

	std::vector<float> cumuDist(n);
	cumuDist[0] = 0.0f;
	for (int i = 1; i < n; i++) {
		cumuDist[i] = cumuDist[i - 1] + glm::distance(bindings[i - 1].contourPoint, bindings[i].contourPoint);
	}

	// blend tip to tip (linear interpolation between two tip)
	int i = 0;
	while (i < n && bindings[i].leafNodeMarker == -1) i++;
	while (i < n) {
		int j = i + 1;
		while (j < n && bindings[j].leafNodeMarker == -1) j++;
		if (j >= n) break;
		float span = cumuDist[j] - cumuDist[i];
		for (int k = i + 1; k < j; k++) {
			float t = (span > 0.0f) ? (cumuDist[k] - cumuDist[i]) / span : 0.0f;
			bindings[k].blendTip = true;
			bindings[k].blendRegionBeginingTip = i;
			bindings[k].blendRegionEndTip = j;
			bindings[k].blendingWeightsTip.resize(2);
			bindings[k].blendingWeightsTip[0] = 1.0f - t;
			bindings[k].blendingWeightsTip[1] = t;
		}
		i = j; // move to the next pair, using j as the new left tip
	}

	// max blending
	using RadiusFn = std::function<float(int, int, int)>;
	auto defaultRadius = [&](int kk, int firstIndex, int lastIndex) {
		float distToStart = cumuDist[kk] - cumuDist[firstIndex];
		float distToEnd = cumuDist[lastIndex] - cumuDist[kk];
		return std::min(distToStart, distToEnd);
		};
	auto normalCompute = [&](int kk, int firstIndex, int lastIndex,
		int& begin, int& end, float& beginPos, float& endPos, float& radiusOut,
		const RadiusFn& radiusFn) {
			float radiusDist = radiusFn(kk, firstIndex, lastIndex);
			radiusOut = radiusDist;

			beginPos = cumuDist[kk] - radiusDist;
			endPos = cumuDist[kk] + radiusDist;

			int idxLeft = static_cast<int>(std::lower_bound(cumuDist.begin(), cumuDist.end(), beginPos) - cumuDist.begin());
			int idxRight = static_cast<int>(std::upper_bound(cumuDist.begin(), cumuDist.end(), endPos) - cumuDist.begin()) - 1;
			begin = std::max(firstIndex, idxLeft);
			end = std::min(lastIndex, idxRight);
		};

	auto assignWeights = [&](int k, int begin, int end, float beginPos, float endPos) {
		float windowLength = endPos - beginPos;
		bindings[k].blendingWeights.resize(end - begin + 1);

		for (int m = begin; m <= end; m++) {
			if (begin == end) {
				bindings[k].blendingWeights[m - begin] = 1.0f;
			}
			else if (m == begin) {
				float outward = cumuDist[m] - beginPos;
				float inward = (cumuDist[m + 1] - cumuDist[m]) / 2.0f;
				bindings[k].blendingWeights[m - begin] = (outward + inward) / windowLength;
			}
			else if (m == end) {
				float inward = (cumuDist[m] - cumuDist[m - 1]) / 2.0f;
				float outward = endPos - cumuDist[m];
				bindings[k].blendingWeights[m - begin] = (inward + outward) / windowLength;
			}
			else {
				float distPrev = cumuDist[m] - cumuDist[m - 1];
				float distNext = cumuDist[m + 1] - cumuDist[m];
				bindings[k].blendingWeights[m - begin] = (distPrev / 2.0f + distNext / 2.0f) / windowLength;
			}
		}
		};

	for (int i = 0; i < n; i++) {
		if (bindings[i].branchingNodeMarker == -1) continue;

		int leafIdx = -1;
		for (int j = 0; j < n; j++) {
			if (bindings[j].leafNodeMarker == bindings[i].bindingAxisID) {
				leafIdx = j;
				break;
			}
		}
		bool isBelow = (leafIdx != -1) && (bindings[i].contourPoint.y < bindings[leafIdx].contourPoint.y);
		int left, right;
		left = i;
		while (left > 0 && bindings[left].leafNodeMarker == -1) left--;
		right = i;
		while (right < n - 1 && bindings[right].leafNodeMarker == -1) right++;

		int windowSum = left + right;
		float distSum = cumuDist[left] + cumuDist[right];

		float prevRadiusDist = -1.0f;
		bool havePrevRadius = false;

		RadiusFn radiusFn = defaultRadius;
		for (int k = left; k <= right; k++) {
			int begin, end;
			float beginPos, endPos, radiusDist;
			normalCompute(k, left, right, begin, end, beginPos, endPos, radiusDist, radiusFn);
			prevRadiusDist = radiusDist;
			havePrevRadius = true;

			bindings[k].blend = true;
			bindings[k].blendRegionBegining = begin;
			bindings[k].blendRegionEnd = end;
			assignWeights(k, begin, end, beginPos, endPos);
		}
	}
	indentationWindow *= 2.f;
	recalculatePrevFrameInverseBlend(bindings);
}


enum IndentationMode : int {
	IndentSymmetricWindow = 0, // original window blending (symmetric window blending)
	MaxWindow = 1, // max window blending (asymmetric window blending that can reach from one tip to another)
	TipAndBranching = 2, // linearly interpolating 2 tips and branching node that is in between
	Tip = 3, // linearly interpolating 2 tips
	
};

void SceneNode::indentationControl(std::vector<ContourBinding>& bindings, int mode) {
	if (bindings.empty()) return;

	const int n = static_cast<int>(bindings.size());

	for (auto& b : bindings) {
		b.blend = false;
		b.blendRegionBegining = -1;
		b.blendRegionEnd = -1;
		b.blendingWeights.clear();
	}

	std::vector<float> cumuDist(n);
	cumuDist[0] = 0.0f;
	for (int i = 1; i < n; i++) {
		cumuDist[i] = cumuDist[i - 1] + glm::distance(bindings[i - 1].contourPoint, bindings[i].contourPoint);
	}

	// 2: blend tip to tip + branching node
	if (mode == TipAndBranching) {
		const float kQPeak = 0.2f; // peak weight given to each branching marker

		int i = 0;
		while (i < n && bindings[i].leafNodeMarker == -1) i++;
		while (i < n) {
			int j = i + 1;
			while (j < n && bindings[j].leafNodeMarker == -1) j++;
			if (j >= n) break;

			// collect all branching markers strictly between i and j, in order
			std::vector<int> markers;
			for (int m = i + 1; m < j; m++) {
				if (bindings[m].branchingNodeMarker != -1) markers.push_back(m);
			}

			float span = cumuDist[j] - cumuDist[i];
			int numMarkers = (int)markers.size();

			std::vector<int> anchors;
			anchors.push_back(i);
			anchors.insert(anchors.end(), markers.begin(), markers.end());
			anchors.push_back(j);

			std::vector<float> anchorT(anchors.size());
			for (size_t a = 0; a < anchors.size(); a++) {
				anchorT[a] = (span > 0.0f) ? (cumuDist[anchors[a]] - cumuDist[i]) / span : 0.0f;
			}

			for (int k = i + 1; k < j; k++) {
				float t = (span > 0.0f) ? (cumuDist[k] - cumuDist[i]) / span : 0.0f;

				std::vector<float> markerW(numMarkers, 0.0f);
				float wSumMarkers = 0.0f;

				for (int r = 0; r < numMarkers; r++) {
					float tPrev = anchorT[r];       // anchor before this marker (i or marker r-1)
					float tHere = anchorT[r + 1];   // this marker
					float tNext = anchorT[r + 2];   // anchor after this marker (marker r+1 or j)

					float w = 0.0f;
					if (t >= tPrev && t <= tHere) {
						float denom = tHere - tPrev;
						w = (denom > 0.0f) ? kQPeak * (t - tPrev) / denom : kQPeak;
					}
					else if (t > tHere && t <= tNext) {
						float denom = tNext - tHere;
						w = (denom > 0.0f) ? kQPeak * (tNext - t) / denom : kQPeak;
					}
					markerW[r] = w;
					wSumMarkers += w;
				}

				float wi = (1.0f - t) * (1.0f - wSumMarkers);
				float wj = t * (1.0f - wSumMarkers);

				bindings[k].blend = true;
				bindings[k].blendRegionBegining = i;
				bindings[k].blendRegionEnd = j;
				bindings[k].blendRegionMid = markers;

				bindings[k].blendingWeights.resize(numMarkers + 2);
				bindings[k].blendingWeights[0] = wi;
				for (int r = 0; r < numMarkers; r++) bindings[k].blendingWeights[r + 1] = markerW[r];
				bindings[k].blendingWeights[numMarkers + 1] = wj;
			}

			i = j;
		}
		recalculatePrevFrameInverseBlend(bindings);
		return;
	}
	else if (mode == Tip) {
		int i = 0;
		while (i < n && bindings[i].leafNodeMarker == -1) i++;
		while (i < n) {
			int j = i + 1;
			while (j < n && bindings[j].leafNodeMarker == -1) j++;
			if (j >= n) break;
			float span = cumuDist[j] - cumuDist[i];
			for (int k = i + 1; k < j; k++) {
				float t = (span > 0.0f) ? (cumuDist[k] - cumuDist[i]) / span : 0.0f;
				bindings[k].blend = true;
				bindings[k].blendRegionBegining = i;
				bindings[k].blendRegionEnd = j;
				bindings[k].blendingWeights.resize(2);
				bindings[k].blendingWeights[0] = 1.0f - t;
				bindings[k].blendingWeights[1] = t;
			}
			i = j; 
		}
		recalculatePrevFrameInverseBlend(bindings);
		return;
	}

	// 0/1: blending
	// 0: normal blending
	// 1: max blending
	using RadiusFn = std::function<float(int, int, int)>;

	auto defaultRadius = [&](int kk, int firstIndex, int lastIndex) {
		float distToStart = cumuDist[kk] - cumuDist[firstIndex];
		float distToEnd = cumuDist[lastIndex] - cumuDist[kk];
		return std::min(distToStart, distToEnd);
		};

	auto normalCompute = [&](int kk, int firstIndex, int lastIndex,
		int& begin, int& end, float& beginPos, float& endPos, float& radiusOut,
		const RadiusFn& radiusFn) {
			float radiusDist = radiusFn(kk, firstIndex, lastIndex);
			radiusOut = radiusDist;

			beginPos = cumuDist[kk] - radiusDist;
			endPos = cumuDist[kk] + radiusDist;

			int idxLeft = static_cast<int>(std::lower_bound(cumuDist.begin(), cumuDist.end(), beginPos) - cumuDist.begin());
			int idxRight = static_cast<int>(std::upper_bound(cumuDist.begin(), cumuDist.end(), endPos) - cumuDist.begin()) - 1;
			begin = std::max(firstIndex, idxLeft);
			end = std::min(lastIndex, idxRight);
		};

	auto assignWeights = [&](int k, int begin, int end, float beginPos, float endPos) {
		float windowLength = endPos - beginPos;
		bindings[k].blendingWeights.resize(end - begin + 1);

		for (int m = begin; m <= end; m++) {
			if (begin == end) {
				bindings[k].blendingWeights[m - begin] = 1.0f;
			}
			else if (m == begin) {
				float outward = cumuDist[m] - beginPos;
				float inward = (cumuDist[m + 1] - cumuDist[m]) / 2.0f;
				bindings[k].blendingWeights[m - begin] = (outward + inward) / windowLength;
			}
			else if (m == end) {
				float inward = (cumuDist[m] - cumuDist[m - 1]) / 2.0f;
				float outward = endPos - cumuDist[m];
				bindings[k].blendingWeights[m - begin] = (inward + outward) / windowLength;
			}
			else {
				float distPrev = cumuDist[m] - cumuDist[m - 1];
				float distNext = cumuDist[m + 1] - cumuDist[m];
				bindings[k].blendingWeights[m - begin] = (distPrev / 2.0f + distNext / 2.0f) / windowLength;
			}
		}
		};

	for (int i = 0; i < n; i++) {
		if (bindings[i].branchingNodeMarker == -1) continue;

		int leafIdx = -1;
		for (int j = 0; j < n; j++) {
			if (bindings[j].leafNodeMarker == bindings[i].bindingAxisID) {
				leafIdx = j;
				break;
			}
		}
		bool isBelow = (leafIdx != -1) && (bindings[i].contourPoint.y < bindings[leafIdx].contourPoint.y);

		int left, right;

		// max blending
		if (mode == MaxWindow) {
			left = i;
			while (left > 0 && bindings[left].leafNodeMarker == -1) left--;

			right = i;
			while (right < n - 1 && bindings[right].leafNodeMarker == -1) right++;
		}
		// normal blending
		else {
			float effectiveDistance = indentationWindow;
			if (leafIdx != -1) {
				float distToLeaf = std::fabs(cumuDist[leafIdx] - cumuDist[i]);
				effectiveDistance = std::min(indentationWindow, distToLeaf);
			}

			float targetStart = cumuDist[i] - effectiveDistance;
			float targetEnd = cumuDist[i] + effectiveDistance;

			left = static_cast<int>(std::lower_bound(cumuDist.begin(), cumuDist.end(), targetStart) - cumuDist.begin());
			left = std::max(0, left);
			right = static_cast<int>(std::upper_bound(cumuDist.begin(), cumuDist.end(), targetEnd) - cumuDist.begin()) - 1;
			right = std::min(n - 1, right);
		}

		int windowSum = left + right;
		float distSum = cumuDist[left] + cumuDist[right];

		float prevRadiusDist = -1.0f;
		bool havePrevRadius = false;

		RadiusFn radiusFn = defaultRadius;
		if (mode == IndentSymmetricWindow) {
			radiusFn = [&](int kk, int firstIndex, int lastIndex) -> float {
				bool nearI = std::fabs(cumuDist[kk] - cumuDist[i]) < (indentationWindow / 8.f);
				if (nearI && havePrevRadius) {
					return prevRadiusDist;
				}
				return defaultRadius(kk, firstIndex, lastIndex);
				};
		}

		for (int k = left; k <= right; k++) {
			int begin, end;
			float beginPos, endPos, radiusDist;

			if (mode == MaxWindow) normalCompute(k, left, right, begin, end, beginPos, endPos, radiusDist, radiusFn);
			else {
				if (!isBelow) {
					normalCompute(k, left, right, begin, end, beginPos, endPos, radiusDist, radiusFn);
				}
				else {
					int kMirror = windowSum - k;
					int mb, me;
					float mbp, mep, mRadiusDist;
					normalCompute(kMirror, left, right, mb, me, mbp, mep, mRadiusDist, radiusFn);
					begin = windowSum - me;
					end = windowSum - mb;
					beginPos = distSum - mep;
					endPos = distSum - mbp;
					radiusDist = mRadiusDist;
				}
			}
			prevRadiusDist = radiusDist;
			havePrevRadius = true;

			bindings[k].blend = true;
			bindings[k].blendRegionBegining = begin;
			bindings[k].blendRegionEnd = end;
			assignWeights(k, begin, end, beginPos, endPos);
		}
	}
	if (mode == MaxWindow) indentationWindow *= 2.f;
	recalculatePrevFrameInverseBlend(bindings);
}

// to animate the contour points when using indentationControl for blending
void SceneNode::animationPerFrameBinding(std::vector<ContourBinding>& bindings) {
	std::vector<glm::mat4> prevInverseSnapshot(bindings.size());
	for (size_t k = 0; k < bindings.size(); k++) {
		prevInverseSnapshot[k] = bindings[k].previousAnimateInverse;
	}

	for (int i = 0; i < bindings.size(); i++) {
		glm::mat4 animatedPosMat = calculateAnimationMatrix(bindings[i]);
		if (i == 0 || i == bindings.size() - 1) continue; // don't animate the first & last points
		if (!bindings[i].blend) {
			bindings[i].contourPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].contourPoint, 1.0f);
		}
		else {

			glm::vec4 animatedPoint = glm::vec4(0.f);
			glm::vec4 closePoint = glm::vec4(0.f);
			glm::mat4 accumulatedT = glm::mat4(0.f);
			float accumulatedWeight = 0.f;
			int tipIndices[2] = { bindings[i].blendRegionBegining, bindings[i].blendRegionEnd };
			if (tipToTipBlending == 0 || tipToTipBlending == 1) {
				int counter = 0;
				// i = current contour point to process
				// j = pointer to the contour point within region of blending for i
				for (int j = bindings[i].blendRegionBegining; j <= bindings[i].blendRegionEnd; j++)
				{
					glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
					glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
					animatedPoint += bindings[i].blendingWeights[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
					counter += 1;
				}
			}
			else {
				if (bindings[i].blendRegionMid.empty()) {
					int tipIndices[2] = { bindings[i].blendRegionBegining, bindings[i].blendRegionEnd };
					for (int counter = 0; counter < 2; counter++)
					{
						int j = tipIndices[counter];
						glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
						glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
						animatedPoint += bindings[i].blendingWeights[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
					}
				}
				else {
					std::vector<int> tipIndices = { bindings[i].blendRegionBegining, bindings[i].blendRegionEnd };

					tipIndices.insert(tipIndices.begin() + 1, bindings[i].blendRegionMid.begin(), bindings[i].blendRegionMid.end());

					for (int counter = 0; counter < tipIndices.size(); counter++)
					{
						int j = tipIndices[counter];
						glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
						glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
						animatedPoint += bindings[i].blendingWeights[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
					}
				}
			}
			bindings[i].contourPoint = animatedPoint;
		}
		// don't use blended transformation for binding position
		bindings[i].closestPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].closestPoint, 1.0f);
	}
	// update previous inverse transformation after updating all points
	for (size_t k = 0; k < bindings.size(); k++) {
		bindings[k].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[k]));
	}
}

// to animate the contour points when using indentationControlMaxAndTipBlending for blending
void SceneNode::animationPerFrameMaxAndTipBlending(std::vector<ContourBinding>& bindings) {
	std::vector<glm::mat4> prevInverseSnapshot(bindings.size());
	for (size_t k = 0; k < bindings.size(); k++) {
		prevInverseSnapshot[k] = bindings[k].previousAnimateInverse;
	}

	for (int i = 0; i < bindings.size(); i++) {
		glm::mat4 animatedPosMat = calculateAnimationMatrix(bindings[i]);
		if (!bindings[i].blend && !bindings[i].blendTip) {
			bindings[i].contourPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].contourPoint, 1.0f);
		}
		else {
			glm::vec4 animatedPoint = glm::vec4(0.f);
			glm::vec4 closePoint = glm::vec4(0.f);
			glm::mat4 accumulatedT = glm::mat4(0.f);
			float accumulatedWeight = 0.f;
			if (bindings[i].blend && !bindings[i].blendTip) {
				int counter = 0;
				// i = current contour point to process
				// j = pointer to the contour point within region of blending for i
				for (int j = bindings[i].blendRegionBegining; j <= bindings[i].blendRegionEnd; j++)
				{
					glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
					glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
					animatedPoint += bindings[i].blendingWeights[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
					counter += 1;
				}
			}
			else if (!bindings[i].blend && bindings[i].blendTip) {
				int tipIndices[2] = { bindings[i].blendRegionBeginingTip, bindings[i].blendRegionEndTip };
				for (int counter = 0; counter < 2; counter++)
				{
					int j = tipIndices[counter];
					glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
					glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
					animatedPoint += bindings[i].blendingWeightsTip[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
				}
			}
			else {
				glm::mat4 accumulatedMatrix1 = glm::mat4(0.f);
				glm::mat4 accumulatedMatrix2 = glm::mat4(0.f);
				int counter = 0;
				glm::vec4 animatedPoint1 = glm::vec4(0.f);
				glm::vec4 animatedPoint2 = glm::vec4(0.f);
				for (int j = bindings[i].blendRegionBegining; j <= bindings[i].blendRegionEnd; j++)
				{
					glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
					glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
					animatedPoint1 += bindings[i].blendingWeights[counter] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
					counter += 1;
				}

				int tipIndices[2] = { bindings[i].blendRegionBeginingTip, bindings[i].blendRegionEndTip };
				for (int counter1 = 0; counter1 < 2; counter1++)
				{
					int j = tipIndices[counter1];
					glm::mat4 jAnimationMatrix = calculateAnimationMatrix(bindings[j]);
					glm::mat4 jPrevFrameAnimationMatrix = prevInverseSnapshot[j];
					animatedPoint2 += bindings[i].blendingWeightsTip[counter1] * jAnimationMatrix * jPrevFrameAnimationMatrix * glm::vec4(bindings[i].contourPoint, 1.0f);
				}
				animatedPoint = 0.5f * animatedPoint1 + 0.5f * animatedPoint2;
			}
			bindings[i].contourPoint = animatedPoint;
		}
		// don't use blended transformation for binding position
		bindings[i].closestPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].closestPoint, 1.0f);
	}
	// update previous inverse transformation after updating all points
	for (size_t k = 0; k < bindings.size(); k++) {
		bindings[k].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[k]));
	}
}
