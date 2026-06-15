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

#include <glm/gtc/quaternion.hpp>
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
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <functional>
#include "toml.hpp"
#include <unordered_set>

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
	return (c1.blending) * c1.childNode->marginTransformation + (1 - (c1.blending)) * c1.parentNode->marginTransformation;
	//return (c1.blending) * (c1.blending) * c1.childNode->marginTransformation + (1 - c1.blending) * (1 - c1.blending) * c1.parentNode->marginTransformation;
}

glm::mat4 calculateAnimationMatrixForNewPoint(float blending, SceneNode* branch) {
	return  blending * branch->marginTransformation + (1 - blending) * branch->parent->marginTransformation;
	//return  blending * blending * branch->marginTransformation + (1 - blending) * (1 - blending) * branch->parent->marginTransformation;
}

glm::vec3 calculateClosestPoint(ContourBinding c1) {
	return (c1.t) * c1.childNode->globalTransformation[3] + (1 - c1.t) * c1.parentNode->globalTransformation[3];
	//return (c1.t * c1.t) * c1.childNode->globalTransformation[3] + (1 - c1.t) * (1 - c1.t) * c1.parentNode->globalTransformation[3];
}

glm::vec3 calculateClosestPointForNewPoint(float t, SceneNode* branch) {
	return t * branch->globalTransformation[3] + (1 - (t)) * branch->parent->globalTransformation[3];
	//return t * t * branch->globalTransformation[3] + (1 - t) * (1 - t) * branch->parent->globalTransformation[3];
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

// to clone the leaf structure if we are not adding new branches (because conditions were not met)
SceneNode* SceneNode::cloneSceneNode(SceneNode* node, SceneNode* parent, std::unordered_map<SceneNode*, SceneNode*>& nodeMap) {
	if (!node) return nullptr;

	SceneNode* copy = new SceneNode();

	copy->localTranslation = node->localTranslation;
	copy->localRotation = node->localRotation;
	copy->localScaling = node->localScaling;

	copy->animateTranslation = node->animateTranslation;
	copy->animateRotation = node->animateRotation;
	copy->animateScaling = node->animateScaling;
	copy->deltatime = node->deltatime;
	copy->animationDirection = node->animationDirection;
	copy->animationAngle = node->animationAngle;
	copy->animationScaling = node->animationScaling;
	copy->animationTime = node->animationTime;
	copy->animationDuration = node->animationDuration;
	copy->S = node->S;
	copy->rotationAngle = node->rotationAngle;
	copy->expansion = node->expansion;
	copy->expansionFactor = node->expansionFactor;
	copy->growth = node->growth;
	copy->growthFactor = node->growthFactor;
	copy->positionOnBranch = node->positionOnBranch;
	copy->axisID = node->axisID;
	copy->branchID = copy->branchID;

	copy->parent = parent;

	// clone and original mapping
	nodeMap[node] = copy;

	for (SceneNode* child : node->children) {
		SceneNode* childCopy = cloneSceneNode(child, copy, nodeMap);
		copy->addChild(childCopy);
	}

	return copy;
}

// to rebind contour points after branching structure modification (split/merge)
std::vector<ContourBinding> SceneNode::rebindContour(const std::vector<ContourBinding>& bindings, const std::unordered_map<SceneNode*, SceneNode*>& nodeMap) {
	std::vector<ContourBinding> reboundBindings;

	for (const ContourBinding& binding : bindings) {
		SceneNode* newParent = nodeMap.at(binding.parentNode);
		SceneNode* newChild = nodeMap.at(binding.childNode);

		reboundBindings.push_back({
			newParent,
			newChild,
			binding.contourPoint,
			binding.t,
			binding.closestPoint,
			binding.previousAnimateInverse,
			binding.newBranchBinding,
			binding.blending
			});
	}
	return reboundBindings;
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
// ------------ toml parser -------------------
static glm::mat4 parseMat4Toml(const toml::array* arr)
{
	glm::mat4 m(1.0f);
	int row = 0;
	for (auto&& r : *arr)
	{
		const auto& rowArr = *r.as_array();
		int col = 0;
		for (auto&& v : rowArr)
		{
			m[col][row] = static_cast<float>(v.as_floating_point()->get()); // col-major: m[col][row]
			col++;
		}
		row++;
	}
	return m;
}

std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> SceneNode::extractEdgeTransformsToml(const std::string& filename)
{
	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> edges;

	toml::table tbl;

	try {
		tbl = toml::parse_file(filename);
	}
	catch (const toml::parse_error& err) {
		std::cerr << "TOML parse error: " << err.description() << "\n";
		return {};
	}

	auto edgeArray = tbl["edge"].as_array();
	if (!edgeArray) {
		std::cerr << "No [edge] entries found\n";
		return {};
	}

	int branchID = 0;
	int axisID = 0;
	for (auto&& edgeVal : *edgeArray)
	{
		auto& edge = *edgeVal.as_table();

		// branch = [parent, child]
		auto branch = *edge["branch"].as_array();
		int parent = static_cast<int>(branch[0].as_integer()->get());
		int child = static_cast<int>(branch[1].as_integer()->get());

		glm::mat4 rotation = parseMat4Toml(edge["rotation"].as_array());
		glm::mat4 scaling = parseMat4Toml(edge["scaling"].as_array());
		glm::mat4 translation = parseMat4Toml(edge["translation"].as_array());

		float scalingFactor = static_cast<float>(edge["scaling_factor"].as_floating_point()->get());
		int rotationDirection = static_cast<int>(edge["rotation_direction"].as_integer()->get());
		float rotationAngle = static_cast<float>(edge["rotation_angle"].as_floating_point()->get());
		float expansionFactor = static_cast<float>(edge["expansion_factor"].as_floating_point()->get());
		float growthFactor = static_cast<float>(edge["growth_factor"].as_floating_point()->get());
		float branchPosition = static_cast<float>(edge["branch_position"].as_floating_point()->get());

		edges.emplace_back(
			parent, child,
			rotation, scaling, translation,
			scalingFactor, rotationDirection,
			rotationAngle, expansionFactor,
			growthFactor, branchPosition,
			axisID, branchID
		);
		axisID++;
	}

	return edges;
}

// ------------ txt parser --------------------
glm::mat4 parseMatrix(std::ifstream& in) {
	glm::mat4 mat(1.0f);
	for (int i = 0; i < 4; i++) {
		std::string line;
		std::getline(in, line);
		std::stringstream ss(line);
		for (int j = 0; j < 4; j++) {
			ss >> mat[j][i]; // column-major order
		}
	}
	return mat;
}

// to build initial branching structure from the python file
// extract the local matrices per edge
std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> SceneNode::extractEdgeTransformsTxt(const std::string& filename) {
	std::ifstream in(filename);
	if (!in.is_open()) {
		std::cerr << "Failed to open file\n";
		return {};
	}

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> edges;
	std::string line;
	std::regex edgeRegex(R"#(Edge\s+(\d+)\s+->\s+(\d+))#");
	std::smatch match;

	// only has information for children! Root is just identity
	// so need to start ID from 1 (0 for root)
	int bID = 0;
	int aID = 0;

	while (std::getline(in, line)) {
		if (std::regex_search(line, match, edgeRegex)) {
			int parent = std::stoi(match[1]);
			int child = std::stoi(match[2]);

			std::getline(in, line); // skip header
			std::getline(in, line);
			glm::mat4 rotation = parseMatrix(in);
			std::getline(in, line);
			glm::mat4 scaling = parseMatrix(in);
			std::getline(in, line);
			glm::mat4 translation = parseMatrix(in);
			std::getline(in, line);
			std::getline(in, line); // scaling factor
			float scalingFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // rotation direction
			float rotationDirection = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // rotation angle
			float rotationAngle = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // expansion factor
			float expansionFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // growth factor
			float growthFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // position on branch
			float branchPosition = std::stof(line);

			edges.emplace_back(parent, child, rotation, scaling, translation, scalingFactor, rotationDirection, rotationAngle, expansionFactor, growthFactor, branchPosition, aID, bID);
			aID++;
		}
	}

	return edges;
}
// ----------------------------------------------------------

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

SceneNode* SceneNode::createBranchingStructure(
	int nodeIndex, std::vector<std::vector<int>> parentChildPairs, std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> transformations) {
	// create node
	SceneNode* node = new SceneNode();
	node->localTranslation = glm::mat4(1.0f);
	node->localRotation = glm::quat(1.0f, 0.f, 0.f, 0.f);
	node->localScaling = glm::mat4(1.0f);
	node->S = 0.f;
	node->animationDirection = 0;
	node->rotationAngle = 0.f;
	node->expansionFactor = 0.f;
	node->growthFactor = 0.f;
	node->positionOnBranch = 0.f;
	node->axisID = 0;
	node->branchID = 0;

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
// end of building branching structure from python file



// implementation of global/relative solution for updating growth rate (at each time step)
void SceneNode::updateGrowthRateForMidNode(SceneNode* root) {
	std::vector<Branch> branches = generateAllBranches(root);
	for (const Branch& b : branches) {
		for (int i = 0; i < b.nodes.size(); i++) {
			bool isMidNode = (i > 0 && i < b.nodes.size() - 1);
			if (isMidNode) {
				SceneNode* root = b.root;
				SceneNode* leaf = b.leaf;

				float d1 = glm::length(glm::vec3(b.nodes[i]->globalTransformation[3]) - glm::vec3(root->globalTransformation[3]));
				float d2 = glm::length(glm::vec3(leaf->globalTransformation[3]) - glm::vec3(b.nodes[i]->globalTransformation[3]));
				float t = d1 / (d1 + d2);
				b.nodes[i]->S = t * leaf->S + (1 - t) * root->S;
				b.nodes[i]->growthFactor = t * leaf->growthFactor + (1 - t) * root->growthFactor;
				//b.nodes[i]->expansionFactor = t * leaf->expansionFactor + (1 - t) * root->expansionFactor;
			}
		}
	}
}

bool animated = false;
// animation of the branching structure
void SceneNode::animate(float deltaTime) {
	animated = true;
	// stop animation after certain time
	if (animationTime >= animationDuration) {
		return;
	}
	animationTime += deltaTime;

	animationAngle += deltaTime * rotationAngle * animationDirection;
	animateRotation = glm::toQuat(glm::rotate(glm::mat4(1.0f), glm::radians(animationAngle), glm::vec3(0, 0, 1)));

	//animationScaling += deltaTime * animationScaling;
	//if (parent == nullptr) animationScaling = (1 + deltaTime * S) * animationScaling;     // S controls growth
	////else animationScaling = (1 + deltaTime * S) * animationScaling;
	//else animationScaling = (1 + deltaTime * (S+parent->S)/2.0) * animationScaling;
	//animateScaling = glm::scale(glm::mat4(1.0f), glm::vec3(animationScaling));

	if (parent == nullptr) animationScaling = (1 + deltaTime * S) * animationScaling;     // S controls growth
	//else animationScaling = (1 + deltaTime * S) * animationScaling;  // not correct representation of growth rate distribution
	//else animationScaling = (1 + deltaTime * (S + parent->S) / 2.0) * animationScaling;
	else animationScaling = (1 + deltaTime * (S * 0.5f + parent->S * 0.5f)) * animationScaling;  // average child and parent because this is branch scaling
	animateScaling = glm::scale(glm::mat4(1.0f), glm::vec3(animationScaling, animationScaling, 1.f));

	if (parent == nullptr) expansionAmount = (1 + deltaTime * expansionFactor * 2.5f) * expansionAmount;        // expansionFactor controls expansion
	//else expansionAmount = (1 + deltaTime * (expansionFactor + parent->expansionFactor) / 2.0) * expansionAmount;
	else expansionAmount = (1 + deltaTime * expansionFactor * 2.5f) * expansionAmount;
	expansion = glm::scale(glm::mat4(1.0f), glm::vec3(expansionAmount, 1.f, 1.f));  // horizontal expansion/expansion in the direction of binding

	//if (parent == nullptr) growthAmount = (1 + deltaTime * growthFactor) * growthAmount;    // controls growth gradient
	//else growthAmount = (1 + deltaTime * growthFactor * 2.5f) * growthAmount;
	//growth = glm::scale(glm::mat4(1.0f), glm::vec3(1.f, growthAmount, 1.f));

	if (parent == nullptr) growthAmount = (1 + deltaTime * growthFactor) * growthAmount;    // vertical growth (needed when add new branch)
	else growthAmount = (1 + deltaTime * (growthFactor * 0.5f + parent->growthFactor * 0.5f)) * growthAmount;
	growth = glm::scale(glm::mat4(1.0f), glm::vec3(1.f, growthAmount, 1.f));

	for (SceneNode* child : children) {
		child->animate(deltaTime);
	}
}

// compute global transformation for all nodes recursively
// to get the final position and draw
void SceneNode::updateBranch(const glm::mat4& parentTransform, const glm::mat4& parentTransformAnimation, const glm::mat4& parentRestInverse, const glm::mat4& parentRest, CPU_Geometry& outGeometry) {
	// convert rotation quaternion back to matrix form
	glm::mat4 animateRotationMatrix = glm::toMat4(animateRotation);  // from quaternion to matrix
	glm::mat4 localRotationMatrix = glm::toMat4(localRotation);      // from quaternion to matrix

	//// with rotation
	//globalTransformation = parentTransform * animateScaling * animateRotationMatrix * localScaling * localRotationMatrix * localTranslation;
	////we don't want scaling to affect the child
	//glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	//temp2[0][0] = 1.0;
	//temp2[1][1] = 1.0;
	//temp2[2][2] = 1.0;
	//glm::mat4 newParentTransform = parentTransform * animateRotationMatrix * localRotationMatrix * temp2;

	//// with rotation, removing scaling
	//glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	//temp2[0][0] = 1.0;
	//temp2[1][1] = 1.0;
	//temp2[2][2] = 1.0;
	//globalTransformation = parentTransform * animateRotationMatrix * localRotationMatrix * temp2;
	////globalTransformation = parentTransform * localRotationMatrix *animateScaling * localScaling * localTranslation;
	//////we don't want scaling to affect the child
	////glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	////temp2[0][0] = 1.0;
	////temp2[1][1] = 1.0;
	////temp2[2][2] = 1.0;
	//glm::mat4 newParentTransform = parentTransform * animateRotationMatrix * localRotationMatrix * temp2;

	// no rotation, removing scaling
	glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	temp2[0][0] = 1.f;
	temp2[1][1] = 1.f;
	temp2[2][2] = 1.f;
	//temp2[3][0] *= 5.f;
	//temp2[3][1] *= 1.75f;
	//temp2[3][2] *= 5.f;

	glm::mat4 temp3 = animateScaling * localScaling * localTranslation;
	temp3[0][0] = 1.0;
	temp3[1][1] = 1.0;
	temp3[2][2] = 1.0;
	//temp3[3][0] = 0;
	//temp3[3][1] = 0;
	//temp3[3][2] = 0;

	marginTransformation = parentTransform * localRotationMatrix * growth * animateScaling * localScaling * localTranslation * expansion;   // transformation for the current node is set here
	//marginTransformation[0][0] = 1.f;
	//marginTransformation[1][1] = 1.f;
	//marginTransformation[2][2] = 1.f;
	globalTransformation = parentTransform * localRotationMatrix * growth * animateScaling * localScaling * localTranslation;
	//globalTransformation[0][0] = 1.f;
	//globalTransformation[1][1] = 1.f;
	//globalTransformation[2][2] = 1.f;
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
	// global position of node
	// for drawing purposes
	glm::vec3 rootPos = glm::vec3(globalTransformation[3]);

	// parent index
	unsigned int currentIndex = outGeometry.verts.size();

	// parent geometry
	outGeometry.verts.push_back(rootPos);
	outGeometry.cols.push_back(glm::vec3(0.f, 0.8f, 0.f));

	for (SceneNode* child : children) {
		// child index to draw line segment from parent to child pair
		unsigned int childIndex = outGeometry.verts.size();

		// parent and child pair (to draw line segment for each pair)
		outGeometry.indices.push_back(currentIndex);
		outGeometry.indices.push_back(childIndex);

		// recurse
		child->updateBranch(newParentTransform, newParentTransformAnimation, restPoseInverse, restPose, outGeometry);
	}
}

void SceneNode::printStructure(SceneNode* node) {
	std::cout << glm::vec3(node->globalTransformation[3]) << std::endl;;
	for (SceneNode* child : node->children) {
		printStructure(child);
	}
}

// positional information function to be used to calculate growth rate based on branch position
// use for bidirectional growth gradient
float positionalInformationFunctionLinear(float positionOnBranch) {
	float info;
	// piecewise function
	if (positionOnBranch >= 0 && positionOnBranch <= 0.4f) info = (2.5f) * positionOnBranch;
	else info = -((5.f / 3.f) * positionOnBranch) + (5.f / 3.f);
	return info;  // position on branch must be [0, 1], if want to increase S rate, multiply directly by scalar
}

//float positionalInformationFunctionSigmoid(float positionOnBranch) {
//	float info;
//	// piecewise function
//	if (positionOnBranch >= 0 && positionOnBranch <= 0.5f) info = (-16.f) * pow(positionOnBranch, 3) + (12.f) * pow(positionOnBranch, 2);
//	else if (positionOnBranch > 0.5f && positionOnBranch <= 1.f) info = (-(16.f) * pow((1.f - positionOnBranch), 3)) + (12.f) * pow((1.f - positionOnBranch), 2);
//	else info = 0.f;
//	return info;  // position on branch must be [0, 1], if want to increase S rate, multiply directly by scalar
//}

float positionalInformationFunctionSigmoid(float positionOnBranch) {
	float info;
	// piecewise function
	if (positionOnBranch >= 0 && positionOnBranch <= 0.4f) info = (-125.f / 4.f) * pow(positionOnBranch, 3) + (75.f / 4.f) * pow(positionOnBranch, 2);
	else if (positionOnBranch > 0.4f && positionOnBranch < 1.f) info = ((125.f / 4.f) * pow(0.4f + ((positionOnBranch - 0.4f) / 1.5f), 3)) - ((225.f / 4.f) * pow(0.4f + ((positionOnBranch - 0.4f) / 1.5f), 2)) + (30 * (0.4f + ((positionOnBranch - 0.4f) / 1.5f))) - 4;
	else info = 0.f;
	return info;  // position on branch must be [0, 1], if want to increase S rate, multiply directly by scalar
}

float inversePositionalInformationFunction(float info) {
	float positionOnBranch;
	if (info >= 0 && info <= 1) {
		if (info <= 1.0f && info <= 0.5f)
			positionOnBranch = info / 2.0f;
		else
			positionOnBranch = 1.0f - (info / 2.0f);
	}
	else {
		// Handle out-of-range input if needed
		positionOnBranch = -1.0f; // or some error value
	}
	return positionOnBranch;
}

// helper function to increment branchID (position along an axis incremented for all descendants of the same axis)
void SceneNode::incrementBranchIDsOnAxis(SceneNode* node, int axisID)
{
	if (node->axisID == axisID)
	{
		node->branchID++;
	}

	for (SceneNode* child : node->children)
	{
		incrementBranchIDsOnAxis(child, axisID);
	}
}

bool SceneNode::divideBranch(SceneNode* node, float threshold, float division, bool bidirectionalGrowth) {
	// recurse on original children
	std::vector<SceneNode*> originalChildren = node->children;
	std::vector<SceneNode*> updatedChildren;
	divided = false;

	for (SceneNode* child : originalChildren) {
		glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
		float distance = glm::length(childPos - parentPos);

		if (distance > threshold) {
			divided = true;

			SceneNode* midNode = new SceneNode();
			glm::vec3 midPos = glm::mix(parentPos, childPos, 1 / division);
			// midNode inherits from child
			midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - parentPos)));
			midNode->localRotation = child->localRotation;
			midNode->S = glm::mix(node->S, child->S, 1 / division);
			midNode->expansionFactor = glm::mix(node->expansionFactor, child->expansionFactor, 1 / division);
			midNode->growthFactor = glm::mix(node->growthFactor, child->growthFactor, 1 / division);
			midNode->positionOnBranch = glm::mix(node->positionOnBranch, child->positionOnBranch, 1 / division);
			if (bidirectionalGrowth) {
				midNode->S = positionalInformationFunctionSigmoid(midNode->positionOnBranch);
				//midNode->expansionFactor = positionalInformationFunctionSigmoid(midNode->positionOnBranch);
				//midNode->expansionFactor *= 0.5f;
			}
			midNode->rotationAngle = child->rotationAngle;
			midNode->animationDirection = child->animationDirection;
			midNode->animationAngle = child->animationAngle;
			midNode->animateRotation = child->animateRotation;
			midNode->animationScaling = 1;
			midNode->animateScaling = glm::mat4(1.0);
			midNode->axisID = node->axisID;
			midNode->branchID = node->branchID + 1;

			incrementBranchIDsOnAxis(child, node->axisID);
			// child set to identity
			child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - childPos)));
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
			if (bidirectionalGrowth) {
				child->S = positionalInformationFunctionSigmoid(child->positionOnBranch);
				//child->expansionFactor = positionalInformationFunctionSigmoid(child->positionOnBranch);
				//child->expansionFactor *= 0.5f;
			}

			midNode->addChild(child);
			midNode->parent = node;
			updatedChildren.push_back(midNode);

			// distinguish as new branch
			node->midBranch = true;
			midNode->midBranch = true;
			child->midBranch = true;
		}
		else {
			updatedChildren.push_back(child);
		}
	}

	node->children = updatedChildren;

	for (SceneNode* child : originalChildren) {
		divided = divided || divideBranch(child, threshold, division, bidirectionalGrowth);
	}
	return divided;
}

//bool SceneNode::divideBranch(SceneNode* node, float threshold, float division, bool bidirectionalGrowth) {
//	for (SceneNode* child : node->children) {
//		glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
//		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
//
//		if (glm::length(childPos - parentPos) > threshold) {
//			glm::vec3 midPos = glm::mix(parentPos, childPos, 1.0f / division);
//
//			SceneNode* midNode = new SceneNode();
//			// midNode inherits from child
//			midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
//			midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - parentPos)));
//			midNode->localRotation = child->localRotation;
//			midNode->S = glm::mix(node->S, child->S, 1 / division);
//			midNode->expansionFactor = glm::mix(node->expansionFactor, child->expansionFactor, 1 / division);
//			midNode->growthFactor = glm::mix(node->growthFactor, child->growthFactor, 1 / division);
//			midNode->positionOnBranch = glm::mix(node->positionOnBranch, child->positionOnBranch, 1 / division);
//			if (bidirectionalGrowth) {
//				midNode->S = positionalInformationFunctionSigmoid(midNode->positionOnBranch);
//				//midNode->expansionFactor = positionalInformationFunctionSigmoid(midNode->positionOnBranch);
//				//midNode->expansionFactor *= 0.5f;
//			}
//			midNode->rotationAngle = child->rotationAngle;
//			midNode->animationDirection = child->animationDirection;
//			midNode->animationAngle = child->animationAngle;
//			midNode->animateRotation = child->animateRotation;
//			midNode->animationScaling = 1;
//			midNode->animateScaling = glm::mat4(1.0);
//			midNode->axisID = node->axisID;
//			midNode->branchID = (node->branchID) + 1;
//
//			incrementBranchIDsOnAxis(child, node->axisID);
//			// child set to identity
//			child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
//			child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - childPos)));
//			child->localRotation = glm::mat4(1.f);
//			child->rotationAngle = 0;
//			child->animationDirection = 0;
//			child->animationAngle = 0.f;
//			child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
//			child->animationScaling = 1;
//			child->animateScaling = glm::mat4(1.0);
//			child->expansion = glm::mat4(1.f);
//			child->expansionAmount = 1.f;
//			child->growth = glm::mat4(1.f);
//			child->growthAmount = 1.f;
//			if (bidirectionalGrowth) {
//				child->S = positionalInformationFunctionSigmoid(child->positionOnBranch);
//				//child->expansionFactor = positionalInformationFunctionSigmoid(child->positionOnBranch);
//				//child->expansionFactor *= 0.5f;
//			}
//
//			// distinguish as new branch
//			node->midBranch = true;
//			midNode->midBranch = true;
//			child->midBranch = true;
//
//			midNode->addChild(child);
//			midNode->parent = node;
//
//			// replace child with midNode in node->children
//			auto it = std::find(node->children.begin(), node->children.end(), child);
//			*it = midNode;
//			divided = true;
//			return true; // stop immediately
//		}
//
//		// no violation here, recurse deeper
//		if (divideBranch(child, threshold, division, bidirectionalGrowth))
//			divided = true;
//			return true; // propagate the early exit up
//	}
//	divided = false;
//	return false;
//}

// find the branch that was divided
void getBranchSegmentsFromBinding(
	SceneNode* current,
	SceneNode* targetChild,
	std::vector<std::pair<SceneNode*, SceneNode*>>& segments,
	bool& found
) {
	if (!current || found) return;

	if (current == targetChild) {
		found = true;
		return;
	}

	// recursively find the divided branch from the parent
	for (SceneNode* child : current->children) {
		segments.push_back({ current, child });
		getBranchSegmentsFromBinding(child, targetChild, segments, found);
		if (found) return;
		// if child path didn't reach targetChild, backtrack
		segments.pop_back();
	}
}

std::tuple<SceneNode*, SceneNode*, float, float, glm::vec3, bool> findClosestPointBranch(std::vector<std::pair<SceneNode*, SceneNode*>>& segments, ContourBinding* contour, bool& found) {
	SceneNode* p;
	SceneNode* c;
	float finalT;
	float finalBlending;

	// only rebind if the point's branch has been divided
	if (contour->childNode->midBranch && contour->parentNode->midBranch) {
		// find the divided segments
		getBranchSegmentsFromBinding(contour->parentNode, contour->childNode, segments, found);
		//std::cout << segments.size() << std::endl;
		// special case, needed for floating point error
		if (contour->t == 0) {
			for (const auto& [parent, child] : segments) {
				if (parent == contour->parentNode) {
					p = parent;
					c = child;
					finalT = contour->t;
					finalBlending = contour->blending;
				}
			}
		}
		else if (contour->t == 1) {
			for (const auto& [parent, child] : segments) {
				if (child == contour->childNode) {
					p = parent;
					c = child;
					finalT = contour->t;
					finalBlending = contour->blending;
				}
			}
		}
		else {
			float final_t = -1.0;
			float final_blending = -1.0;
			float eps[] = { 1e-4f, 1e-3f, 1e-2f, 1e-1f };   // floating point error keeps increasing
			for (int i = 0; i < 4; i++) {
				if (final_t > -1.0) break;
				for (const auto& [parent, child] : segments) {
					float distance = FLT_MAX;
					glm::vec3 p1 = glm::vec3(parent->globalTransformation[3]);
					glm::vec3 p2 = glm::vec3(child->globalTransformation[3]);
					glm::vec3 dir = p2 - p1;
					//if (contour->closestPoint.y > p2.y) contour->closestPoint.y = p2.y;

					float t = calculateT(p1, p2, contour->closestPoint);
					if (t < -eps[i] || t > 1.0f + eps[i]) continue;
					t = glm::clamp(t, 0.0f, 1.0f);
					final_t = t;

					float blending = calculateBlending(p1, p2, contour->closestPoint);
					if (blending < -eps[i] || blending > 1.0f + eps[i]) continue;
					blending = glm::clamp(blending, 0.0f, 1.0f);
					final_blending = blending;

					p = parent;
					c = child;
					finalT = final_t;
					finalBlending = final_blending;
				}
			}
		}
	}
	// otherwise, keep it the same
	else {
		p = contour->parentNode;
		c = contour->childNode;
		finalT = contour->t;
		finalBlending = contour->blending;
	}

	return std::make_tuple(p, c, finalT, finalBlending, contour->closestPoint, found);
}

void SceneNode::rebindContourWithBrokenBranch(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments, int& i, std::vector<ContourBinding>& bindings) {
	i = 0;
	for (ContourBinding& binding : bindings) {
		// get the current closest point, find which branch to bind to
		bool found = false;
		std::tuple<SceneNode*, SceneNode*, float, float, glm::vec3, bool> branch = findClosestPointBranch(segments, &binding, found);
		// guarantee t > 0 
		if (std::get<2>(branch) == 0 && std::get<0>(branch)->parent != NULL) { // or could do binding.t != 0 instead of checking the parent 
			binding.parentNode = std::get<0>(branch)->parent;
			binding.childNode = std::get<0>(branch);
			binding.blending = 1;
			binding.t = 1;
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			binding.closestPoint = calculateClosestPoint(binding);
			//continue;
		}
		// only rebind for points that are bound to subdivided branch
		else if (std::get<5>(branch)) {
			binding.parentNode = std::get<0>(branch);
			binding.childNode = std::get<1>(branch);
			binding.t = std::get<2>(branch);
			binding.blending = std::get<3>(branch);
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			binding.closestPoint = calculateClosestPoint(binding);
		}
		// also need to adjust the parent binding of the points that are children to subdivided branch (only the parentNode's animateMatrix change)
		// parent binding is the subdivided branch
		else if (binding.parentNode->midBranch && !binding.childNode->midBranch) {
			binding.parentNode = binding.childNode->parent;
			binding.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(binding));
			binding.closestPoint = calculateClosestPoint(binding);
		}
		i++;
		segments.clear();
	}
}

// helper function to decrement branchID for all descendants on the same axis
void SceneNode::decrementBranchIDsOnAxis(SceneNode* node, int axisID)
{
	if (node->axisID == axisID)
	{
		node->branchID--;
	}

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
				// if bidirectional growth, S and growthFactor must be handled separately
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
bool isPointOnSegment(glm::vec2 A, glm::vec2 B, glm::vec2 P) {
	float cross = (P.x - A.x) * (B.y - A.y) - (P.y - A.y) * (B.x - A.x);
	if (fabs(cross) > 1e-6) return false;  // not collinear

	float dot = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);
	if (dot < 0) return false; // before A

	float lenSq = (B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y);
	if (dot > lenSq) return false; // after B

	return true;
}

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

// ADD NEW BRANCH
// probably need a better way to find the contour point to add a new branch to, currently also can add new branch by clicking on the contour point
ContourBinding* SceneNode::findContourPointToAddBranch(float height, SceneNode* root, std::vector<ContourBinding>& contourPoints) {
	ContourBinding* finalContour = nullptr;
	//std::vector<ContourBinding*> finalContours;
	float difference = FLT_MAX;

	// don't want to connect with the first and last 
	for (size_t i = contourPoints.size() - 1; i > 1; i--) {
		ContourBinding& contour = contourPoints[i];
		float dist = abs(glm::length(glm::vec3(root->globalTransformation[3]) - contour.contourPoint) - height);
		if (dist < difference) {
			difference = dist;
			finalContour = &contour;
		}
	}

	return finalContour;
}

// finds the best location on any existing branch to attach a new branch
// visits every parent-child relationship
// so it is not necessarily on the branch that contourPoint is bound to
ContourBinding SceneNode::findBestBinding(SceneNode* root, const glm::vec3& contourPoint) {
	ContourBinding bestBinding;
	float minTotalDistance = std::numeric_limits<float>::max();

	// just consider main axis for now (one)
	std::function<void(SceneNode*)> dfs = [&](SceneNode* node) {
		for (SceneNode* child : node->children) {
			glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
			glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
			glm::vec3 dir = childPos - parentPos;

			const int steps = 50;
			for (int i = 0; i <= steps; i++) {
				float t = static_cast<float>(i) / steps;
				glm::vec3 projected = parentPos + t * dir;

				// distance from contour point to the test point on the branch
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
				if (totalDistance < minTotalDistance && projected != glm::vec3(0.f, 0.f, 0.f)) {   // don't want to add a new branch from the root
					minTotalDistance = totalDistance;

					if (t == 1) {     // special case for tip of the branch, to preserve the tip of the main axis
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

bool SceneNode::splitAtBinding(ContourBinding* pointToBreak)
{
	std::vector<SceneNode*> originalChildren = pointToBreak->parentNode->children;
	std::vector<SceneNode*> updatedChildren;

	SceneNode* node = pointToBreak->parentNode;
	SceneNode* child = pointToBreak->childNode;

	if (!node || !child) {
		divided = false;
		return divided;
	}
	else {
		glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
		float distance = glm::length(childPos - parentPos);
		SceneNode* midNode = new SceneNode();

		// break in the closest point of the branch found in bindBestBinding (the projection of the point that minimizes totalDistance)
		midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
		midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(pointToBreak->closestPoint - parentPos)));
		midNode->localRotation = child->localRotation;
		midNode->S = glm::mix(node->S, child->S, (glm::length(pointToBreak->closestPoint - parentPos) / distance));
		//midNode->S = child->S;
		midNode->expansionFactor = glm::mix(node->expansionFactor, child->expansionFactor, (glm::length(pointToBreak->closestPoint - parentPos) / distance));
		midNode->growthFactor = glm::mix(node->growthFactor, child->growthFactor, (glm::length(pointToBreak->closestPoint - parentPos) / distance));
		midNode->positionOnBranch = glm::mix(node->positionOnBranch, child->positionOnBranch, (glm::length(pointToBreak->closestPoint - parentPos) / distance));
		midNode->rotationAngle = child->rotationAngle;
		midNode->animationDirection = child->animationDirection;
		midNode->animationAngle = child->animationAngle;
		midNode->animateRotation = child->animateRotation;
		midNode->animationScaling = 1;
		midNode->animateScaling = glm::mat4(1.0);
		midNode->axisID = node->axisID;
		midNode->branchID = node->branchID + 1;
		incrementBranchIDsOnAxis(child, node->axisID);
		// this function is simply for dividng to add branch; no need to consider different growth gradient

		// child set to identity
		child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
		child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(childPos - pointToBreak->closestPoint)));
		child->localRotation = glm::mat4(1.f);
		child->rotationAngle = 0;
		child->animationDirection = 0;
		child->animationAngle = 0.f;
		child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		child->animationScaling = 1;
		child->animateScaling = glm::mat4(1.0);
		//child->positionOnBranch = positionalInformaionFunction(child->positionOnBranch);
		//child->expansion = glm::mat4(1.f);
		//child->growth = glm::mat4(1.f);

		midNode->addChild(child);
		midNode->parent = node;
		//updatedChildren.push_back(midNode);

		// distinguish as new branch
		node->midBranch = true;
		midNode->midBranch = true;
		child->midBranch = true;

		node->trackOriginalBranch = true;   // parent of new node
		child->trackOriginalBranch = true;   // child of new node
		// to rebind all contour points that are binded to these branches; need to exclusively include the contour's original branch because the new branch might be added to a different branch
		// so if we don't include the contour's original binded branch, the order of rebinding might get messed up
		pointToBreak->parentNode->trackOriginalBranch = true;
		pointToBreak->childNode->trackOriginalBranch = true;

		// to add new branch at midNode
		midNode->addBranch = true;

		// update child list for parent node to have the new split point (instead of the original child)
		// 1. First fix node->children, replacing child with midNode
		for (SceneNode* c : originalChildren) {
			if (c == child) {
				updatedChildren.push_back(midNode);
			}
			else {
				updatedChildren.push_back(c);
			}
		}
		node->children = updatedChildren;

		//// 2. Then wire midNode into the tree
		//midNode->parent = node;
		//midNode->children.push_back(child);  // bypass addChild to avoid side effects
		//child->parent = midNode;

		divided = true;
		return divided;
	}
}

bool SceneNode::divideBranchMinDistance(SceneNode* node, ContourBinding* contour) {
	// recurse on original children
	std::vector<SceneNode*> originalChildren = node->children;
	std::vector<SceneNode*> updatedChildren;

	divided = false;

	// find the point to break (and the corresponding branch)
	ContourBinding pointToBreak = findBestBinding(node, contour->contourPoint);

	for (SceneNode* child : originalChildren) {
		glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
		float distance = glm::length(childPos - parentPos);

		// find the branch that the contour is binded to
		if (node->globalTransformation[3] == pointToBreak.parentNode->globalTransformation[3] && child->globalTransformation[3] == pointToBreak.childNode->globalTransformation[3]) {
			divided = true;

			SceneNode* midNode = new SceneNode();

			// break in the closest point of the branch found in bindBestBinding (the projection of the point that minimizes totalDistance)
			midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(pointToBreak.closestPoint - parentPos)));
			midNode->localRotation = child->localRotation;
			midNode->S = glm::mix(child->parent->S, child->S, (glm::length(pointToBreak.closestPoint - parentPos) / distance));
			//midNode->S = child->S;
			midNode->expansionFactor = glm::mix(child->parent->expansionFactor, child->expansionFactor, (glm::length(pointToBreak.closestPoint - parentPos) / distance));
			midNode->growthFactor = glm::mix(child->parent->growthFactor, child->growthFactor, (glm::length(pointToBreak.closestPoint - parentPos) / distance));
			midNode->positionOnBranch = glm::mix(child->parent->positionOnBranch, child->positionOnBranch, (glm::length(pointToBreak.closestPoint - parentPos) / distance));
			midNode->rotationAngle = child->rotationAngle;
			midNode->animationDirection = child->animationDirection;
			midNode->animationAngle = child->animationAngle;
			midNode->animateRotation = child->animateRotation;
			midNode->animationScaling = 1;
			midNode->animateScaling = glm::mat4(1.0);
			// this function is simply for dividng to add branch; no need to consider different growth gradient

			// child set to identity
			child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(childPos - pointToBreak.closestPoint)));
			child->localRotation = glm::mat4(1.f);
			child->rotationAngle = 0;
			child->animationDirection = 0;
			child->animationAngle = 0.f;
			child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
			child->animationScaling = 1;
			child->animateScaling = glm::mat4(1.0);
			//child->positionOnBranch = positionalInformaionFunction(child->positionOnBranch);
			//child->expansion = glm::mat4(1.f);
			//child->growth = glm::mat4(1.f);

			midNode->addChild(child);
			midNode->parent = node;
			updatedChildren.push_back(midNode);
			// distinguish as new branch
			node->midBranch = true;
			midNode->midBranch = true;
			child->midBranch = true;

			node->trackOriginalBranch = true;   // parent of new node
			child->trackOriginalBranch = true;   // child of new node
			// to rebind all contour points that are binded to these branches; need to exclusively include the contour's original branch because the new branch might be added to a different branch
			// so if we don't include the contour's original binded branch, the order of rebinding might get messed up
			contour->parentNode->trackOriginalBranch = true;
			contour->childNode->trackOriginalBranch = true;

			// to add new branch at midNode
			midNode->addBranch = true;
		}
		else {
			updatedChildren.push_back(child);
		}
	}

	node->children = updatedChildren;

	for (SceneNode* child : originalChildren) {
		divided = divided || divideBranchMinDistance(child, contour);
	}
	return divided;
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

// find the branch that was divided, add a new branch
SceneNode* SceneNode::addNewBranch(SceneNode* node, ContourBinding* contour, int& maxID) {
	if (node->addBranch) {
		// found the midNode that we divided, add a new branch from here
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

		// might make more sense to set these values to the tip of the main axis values (like 1 or 2, not a multiple of its parent)
		// because the value will be small with subdivision
		newNode->S = node->S * 2;   // controls how fast or slow this new node will grow, needs to be extremely big for the new branch to grow fast enough cuz node->S is too small
		newNode->expansionFactor = node->expansionFactor * 1;
		newNode->growthFactor = node->growthFactor * 2; // same reasoning as S
		newNode->positionOnBranch = 1.f;
		newNode->animationDirection = node->animationDirection;
		// for S and growthFactor, since newNode's parameter is inherited from node's (which is way less than 1), in order for newNode to grow fast enough, it has to be multiplied by at least 10 (to make it bigger than 1)
		// also has to do with the fact that the newly added branch is way shorter compared to the main axis
		newNode->axisID = maxID + 1;
		//std::cout << newNode->axisID << std::endl;
		newNode->branchID = 0;

		// Connect to tree
		newNode->parent = node;
		node->addChild(newNode);
		node->addBranch = false;
		// to add new branch
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

// finds neighboring contour bindings around a given binding c
std::vector<ContourBinding*> getNearbyBindings(ContourBinding* c, std::vector<ContourBinding>& bindings, float distance) {
	std::vector<ContourBinding*> result;

	int index = -1;
	for (int i = 0; i < bindings.size(); i++) {
		if (bindings[i].contourPoint == c->contourPoint) {
			index = i;
			break;
		}
	}

	for (int i = index - 1; i >= 0; i--) {
		float dist = glm::distance(bindings[i].contourPoint, c->contourPoint);
		if (dist <= distance) {
			result.push_back(&bindings[i]);
		}
		else {
			break;
		}
	}

	std::reverse(result.begin(), result.end());
	result.push_back(c);

	for (int i = index + 1; i < static_cast<int>(bindings.size()); i++) {
		float dist = glm::distance(bindings[i].contourPoint, c->contourPoint);
		if (dist <= distance) {
			result.push_back(&bindings[i]);
		}
		else {
			break;
		}
	}

	return result;
}

// rebind all the contour points that were bound to the broken branch to the new branch
void SceneNode::rebindToNewBranch(SceneNode* newNode, ContourBinding* contour, std::vector<ContourBinding>& bindings, float dist) {
	std::vector<ContourBinding*> toRebind = getNearbyBindings(contour, bindings, dist);
	// find the exact point to bind to the new branch
	int index = -1;
	int leftMost = -1;
	int rightMost = -1;
	ContourBinding newLeftPoint;
	ContourBinding newRightPoint;

	for (int i = 0; i < toRebind.size(); i++) {
		if (toRebind[i]->contourPoint == contour->contourPoint) {
			index = i;   // index is tip point
			break;
		}
	}
	for (int i = 0; i < bindings.size(); i++) {
		if (bindings[i].contourPoint == toRebind[0]->contourPoint) {
			leftMost = i;
		}
		else if (bindings[i].contourPoint == toRebind[toRebind.size() - 1]->contourPoint)
			rightMost = i;
	}

	if (index != -1) {
		float leftContourDistance = glm::length(toRebind[0]->contourPoint - toRebind[index]->contourPoint);
		for (int i = 0; i < index; i++) {
			if (i == 0 && toRebind[i]->t == 1) { // first point is bound to a different branching node
				newLeftPoint.parentNode = newNode->parent->parent;
				newLeftPoint.childNode = newNode->parent;
				newLeftPoint.blending = 1.f;
				newLeftPoint.t = 1.f;
				newLeftPoint.closestPoint = calculateClosestPoint(newLeftPoint);
				newLeftPoint.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(newLeftPoint));
				newLeftPoint.newBranchBinding = true;
				continue;
			}
			else if (toRebind[i]->t != 1) {
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
				if ((toRebind[i]->t == 0 || toRebind[i]->blending == 0) && newNode->parent->parent != NULL) {
					toRebind[i]->parentNode = newNode->parent->parent;
					toRebind[i]->childNode = newNode->parent;
					toRebind[i]->blending = 1;
					toRebind[i]->t = 1;
					toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
					toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
					toRebind[i]->newBranchBinding = true;
				}
			}
		}

		float rightContourDistance = glm::length(toRebind[toRebind.size() - 1]->contourPoint - toRebind[index]->contourPoint);
		for (int i = index + 1; i < toRebind.size(); i++) {
			if (i == toRebind.size() - 1 && toRebind[i]->t == 1) { // last point is bound to a different branching node
				newRightPoint.parentNode = newNode->parent->parent;
				newRightPoint.childNode = newNode->parent;
				newRightPoint.blending = 1.f;
				newRightPoint.t = 1.f;
				newRightPoint.closestPoint = calculateClosestPoint(newRightPoint);
				newRightPoint.previousAnimateInverse = glm::inverse(calculateAnimationMatrix(newRightPoint));
				newRightPoint.newBranchBinding = true;
				continue;
			}
			if (toRebind[i]->t != 1) {
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
				if ((toRebind[i]->t == 0 || toRebind[i]->blending == 0) && newNode->parent->parent != NULL) {
					toRebind[i]->parentNode = newNode->parent->parent;
					toRebind[i]->childNode = newNode->parent;
					toRebind[i]->blending = 1;
					toRebind[i]->t = 1;
					toRebind[i]->closestPoint = calculateClosestPoint(*toRebind[i]);
					toRebind[i]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[i]));
					toRebind[i]->newBranchBinding = true;
				}
			}
		}
		toRebind[index]->parentNode = newNode->parent;
		toRebind[index]->childNode = newNode;
		toRebind[index]->blending = 1;
		toRebind[index]->t = 1;
		toRebind[index]->closestPoint = calculateClosestPoint(*toRebind[index]);
		toRebind[index]->previousAnimateInverse = glm::inverse(calculateAnimationMatrix(*toRebind[index]));
		toRebind[index]->newBranchBinding = true;
	}
	
	//if (leftMost != -1) {
	//	std::cout << leftMost << std::endl;
	//	bindings.insert(bindings.begin() + leftMost, newLeftPoint);
	//}
	//if (rightMost != -1) {
	//	std::cout << rightMost << std::endl;
	//	bindings.insert(bindings.begin() + rightMost + 1, newRightPoint);
	//}
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
// -----------

static bool isBranchBoundary(SceneNode* node)
{
	return node->children.empty() || node->children.size() > 1;  // (leaf or branching node)
}

static SceneNode* collectBranchEdges(
	SceneNode* branchStart,
	SceneNode* firstChild,
	std::vector<std::pair<SceneNode*, SceneNode*>>& branchEdges)
{
	SceneNode* parent = branchStart;
	SceneNode* cur = firstChild;

	while (true)
	{
		branchEdges.push_back({ parent, cur });

		if (isBranchBoundary(cur)) {
			return cur; // return branching/leaf node
		}
		parent = cur;
		cur = cur->children[0];
	}
}

static void validateBranchBindings(
	SceneNode* branchStart,
	SceneNode* branchEnd,
	const std::vector<std::pair<SceneNode*, SceneNode*>>& branchEdges,
	const std::vector<ContourBinding>& bindings,
	int& contourPos,
	std::vector<std::pair<int, BranchKey>>& misorientedPoints)
{
	BranchKey currentBranch{ branchStart, branchEnd };

	while (contourPos < (int)bindings.size())
	{
		const ContourBinding& b = bindings[contourPos];
		bool isEndMarker = (std::abs(b.t - 1.0f) < 1e-6f);

		if (!isEndMarker)
		{
			bool onBranch = false;
			for (const auto& edge : branchEdges)
			{
				if (b.parentNode == edge.first && b.childNode == edge.second)
				{
					onBranch = true;
					break;
				}
			}

			if (!onBranch)
			{
				std::cout << "contour index: " << contourPos << std::endl;
				std::cout << "contour point: " << bindings[contourPos].contourPoint << std::endl;
				std::cout << "branch start: " << glm::vec3(branchStart->globalTransformation[3]) << std::endl;
				std::cout << "branch end: " << glm::vec3(branchEnd->globalTransformation[3]) << std::endl;
				std::cout << "contour's parent node: " << glm::vec3(b.parentNode->globalTransformation[3]) << std::endl;
				std::cout << "contour's child node: " << glm::vec3(b.childNode->globalTransformation[3]) << std::endl;
				misorientedPoints.push_back({ contourPos, currentBranch });
			}
			contourPos++;
		}
		else
		{
			contourPos++;
			break;
		}
	}
}

void SceneNode::validateBindingsDFS(
	SceneNode* node,
	const std::vector<ContourBinding>& bindings,
	int& contourPos,
	std::vector<std::pair<int, BranchKey>>& misorientedPoints)
{
	if (!node)
		return;
	for (SceneNode* child : node->children)
	{
		std::vector<std::pair<SceneNode*, SceneNode*>> branchEdges;
		SceneNode* branchEnd = collectBranchEdges(node, child, branchEdges);

		validateBranchBindings(node, branchEnd, branchEdges, bindings, contourPos, misorientedPoints);
		validateBindingsDFS(branchEnd, bindings, contourPos, misorientedPoints);
		validateBranchBindings(node, branchEnd, branchEdges, bindings, contourPos, misorientedPoints);
	}
}

// find cross over
std::vector<std::pair<int, BranchKey>> SceneNode::findMisorientedContourIndices(
	SceneNode* root,
	const std::vector<ContourBinding>& bindings)
{
	std::vector<std::pair<int, BranchKey>> misorientedPoints;
	int contourPos = 0;
	validateBindingsDFS(root, bindings, contourPos, misorientedPoints);
	return misorientedPoints;
}

//// Recursive traversal that validates contour bindings in traversal order
//// contourPos: current scan position in the bindings vector (updated as we go)
//void SceneNode::validateBindingsDFS(
//	SceneNode* node,
//	const std::vector<ContourBinding>& bindings,
//	int& contourPos,
//	std::vector<std::pair<int, BranchKey>>& misorientedPoints)
//{
//	if (!node)
//		return;
//
//	for (SceneNode* child : node->children)
//	{
//		if (child->children.size() > 1 || child->children.size() == 0) { // leaf/branching node detected
//
//		}
//		BranchKey currentBranch{ node, child };
//
//		int blockStart = contourPos;
//
//		//std::cout
//		//	<< "DOWN "
//		//	<< glm::vec3(node->globalTransformation[3])
//		//	<< " -> "
//		//	<< glm::vec3(child->globalTransformation[3])
//		//	<< '\n';
//
//		while (contourPos < (int)bindings.size())
//		{
//			const ContourBinding& b = bindings[contourPos];
//			bool isEndMarker =
//				(std::abs(b.t - 1.0f) < 1e-6f);
//
//			if (!isEndMarker)
//			{
//				// Interior point — must be bound to currentBranch
//				if (b.parentNode != node || b.childNode != child) {
//					std::cout << "contour index: " << contourPos << std::endl;
//					std::cout << "contour point: " << bindings[contourPos].contourPoint << std::endl;
//					std::cout << "parent node: " << glm::vec3(node->globalTransformation[3]) << std::endl;
//					std::cout << "contour's parent node: " << glm::vec3(b.parentNode->globalTransformation[3]) << std::endl;
//					std::cout << "child node: " << glm::vec3(child->globalTransformation[3]) << std::endl;
//					std::cout << "contour's child node: " << glm::vec3(b.childNode->globalTransformation[3]) << std::endl;
//					misorientedPoints.push_back({ contourPos, currentBranch });
//				}
//				contourPos++;
//			}
//			else
//			{
//				// End-marker found — advance past it and stop scanning this block.
//				//std::cout << contourPos << std::endl;
//				contourPos++;
//				break;
//			}
//		}
//
//		// Recurse into child's subtree between the Down and Up passes.
//		validateBindingsDFS(child, bindings, contourPos, misorientedPoints);
//
//		//std::cout
//		//	<< "UP   "
//		//	<< glm::vec3(node->globalTransformation[3])
//		//	<< " -> "
//		//	<< glm::vec3(child->globalTransformation[3])
//		//	<< '\n';
//
//		while (contourPos < (int)bindings.size())
//		{
//			const ContourBinding& b = bindings[contourPos];
//			bool isEndMarker =
//				(std::abs(b.t - 1.0f) < 1e-6f);
//
//			if (!isEndMarker)
//			{
//				if (b.parentNode != node || b.childNode != child) {
//					std::cout << "contour index: " << contourPos << std::endl;
//					std::cout << "contour point: " << bindings[contourPos].contourPoint << std::endl;
//					std::cout << "parent node: " << glm::vec3(node->globalTransformation[3]) << std::endl;
//					std::cout << "contour's parent node: " << glm::vec3(b.parentNode->globalTransformation[3]) << std::endl;
//					std::cout << "child node: " << glm::vec3(child->globalTransformation[3]) << std::endl;
//					std::cout << "contour's child node: " << glm::vec3(b.childNode->globalTransformation[3]) << std::endl;
//					misorientedPoints.push_back({ contourPos, currentBranch });
//				}
//				contourPos++;
//			}
//			else
//			{
//				//std::cout << contourPos << std::endl;
//				contourPos++;
//				break;
//			}
//		}
//	}
//}

//// Entry point
//std::vector<std::pair<int, BranchKey>> SceneNode::findMisorientedContourIndices(
//	SceneNode* root,
//	const std::vector<ContourBinding>& bindings)
//{
//	std::vector<ContourBinding> firstHalf(
//		bindings.begin(),
//		bindings.begin() + bindings.size() / 2);
//
//	std::vector<std::pair<int, BranchKey>> misorientedPoints;
//	int contourPos = 0;
//	validateBindingsDFS(root, firstHalf, contourPos, misorientedPoints);
//	return misorientedPoints;
//}

// NO NEED TO USE NOW
std::vector<size_t> SceneNode::contourBindingIndicesToRebind(const std::vector<ContourBinding>& bindings, SceneNode* root) {
	std::vector<size_t> indices;
	for (size_t i = 1; i < bindings.size() - 1; i++) {
		if (bindings[i].childNode->trackOriginalBranch && bindings[i].parentNode->trackOriginalBranch) {
			indices.push_back(i);
			if (bindings[i].newBranchBinding) return {};  // do not want to add new branch if point belongs to previous new branch; return empty vector
		}
	}
	return indices;
}

// returns the index of the contour point that will be binded to the branching point
std::pair<int, int> branchingPointContourPoint(float leftContourLength, float rightContourLength, float initialLeftLength, float initialRightLength, int division, std::vector<size_t> left, std::vector<size_t> right, std::vector<ContourBinding>& bindings) {
	if (!left.empty() && !right.empty()) {
		// take the smaller ratio out of the two (left and right)
		float leftDistance = leftContourLength / division;
		float rightDistance = rightContourLength / division;
		float smallerDistance = (leftDistance < rightDistance) ? leftDistance : rightDistance;
		int leftIndex;
		int rightIndex;

		float finalDistance = FLT_MAX;
		float accumulatedDistance = initialLeftLength;
		for (int i = left.size() - 1; i > 0; i--) {
			accumulatedDistance += (glm::length(bindings[left[i]].contourPoint - bindings[left[i - 1]].contourPoint));
			if (abs(smallerDistance - accumulatedDistance) < finalDistance) {
				finalDistance = abs(smallerDistance - accumulatedDistance);
				leftIndex = i;
			}
		}

		finalDistance = FLT_MAX;
		accumulatedDistance = initialRightLength;
		for (int i = 0; i < right.size() - 1; i++) {
			accumulatedDistance += (glm::length(bindings[right[i + 1]].contourPoint - bindings[right[i]].contourPoint));
			if (abs(smallerDistance - accumulatedDistance) < finalDistance) {
				finalDistance = abs(smallerDistance - accumulatedDistance);
				rightIndex = i;
			}
		}
		return std::make_pair(leftIndex, rightIndex);
	}
	else return std::make_pair(-1, -1);
}

void SceneNode::rebindContourToNewBranchIndexBased(SceneNode* newNode, ContourBinding* contour, int division, std::vector<size_t>& toRebind, std::vector<ContourBinding>& bindings) {
	// find the contour points to rebind
	std::vector<size_t> rebind, leftRebind, rightRebind;
	bool leftSide = false;
	bool rightSide = false;

	//for (size_t i : toRebind) {  // divide based on x values
	//	if (contour->contourPoint == bindings[i].contourPoint) {
	//		for (size_t i2 : toRebind) {
	//			if (bindings[i2].contourPoint.x < 0) {
	//				leftRebind.push_back(i2);
	//			}
	//			else if (bindings[i2].contourPoint.x > 0) {
	//				rightRebind.push_back(i2);
	//			}
	//		}
	//		if (contour->contourPoint.x < 0) {
	//			rebind = leftRebind;
	//			leftSide = true;
	//		}
	//		else {
	//			rebind = rightRebind;
	//			rightSide = true;
	//		}
	//		break; // found match
	//	}
	//}

	glm::vec3 parentBranch = glm::normalize(newNode->parent->parent->globalTransformation[3] - newNode->parent->globalTransformation[3]);
	glm::vec3 newBranch = glm::normalize(newNode->parent->globalTransformation[3] - newNode->globalTransformation[3]);
	float cross = newBranch.x * parentBranch.y - newBranch.y * parentBranch.x;
	for (size_t i : toRebind) {
		if (contour->contourPoint == bindings[i].contourPoint) {
			for (size_t i2 : toRebind) {
				glm::vec3 vec = glm::normalize(bindings[i2].contourPoint - glm::vec3(newNode->parent->globalTransformation[3]));
				float side = parentBranch.x * vec.y - parentBranch.y * vec.x;   // 2d cross product
				if (side < 0) {
					leftRebind.push_back(i2);
				}
				else if (side > 0) {
					rightRebind.push_back(i2);
				}
			}
			if (cross < 0) {
				rebind = leftRebind;
				leftSide = true;
			}
			else if (cross > 0) {
				rebind = rightRebind;
				rightSide = true;
			}
			break;
		}
	}

	// divide the group of points to the left and right (not including the contour itself)
	std::vector<size_t> left;
	std::vector<size_t> right;
	int index = -1;
	int leftBranchingPointIndex = -1;
	int rightBranchingPointIndex = -1;
	for (size_t i : rebind) {
		if (bindings[i].contourPoint == contour->contourPoint) {
			index = i;
			break;
		}
	}
	if (index != -1) {
		auto it = std::find(rebind.begin(), rebind.end(), index);
		if (it != rebind.end()) {
			left.assign(rebind.begin(), it);
			right.assign(it + 1, rebind.end());
		}
	}

	// rebind the contour points
	// branching point binding is determined by distance
	float branchLength = glm::length(newNode->globalTransformation[3] - newNode->parent->globalTransformation[3]);
	float leftContourLength = (!left.empty()) ? glm::length(bindings[left.front()].contourPoint - bindings[index].contourPoint) : 0.f;
	float rightContourLength = (!right.empty()) ? glm::length(bindings[index].contourPoint - bindings[right.back()].contourPoint) : 0.f;
	float initialLeftLength = (!left.empty()) ? glm::length(bindings[left.back()].contourPoint - bindings[index].contourPoint) : 0.f;
	float initialRightLength = (!right.empty()) ? glm::length(bindings[index].contourPoint - bindings[right.front()].contourPoint) : 0.f;
	std::pair<int, int> branchingPointIndex = branchingPointContourPoint(leftContourLength, rightContourLength, initialLeftLength, initialRightLength, division, left, right, bindings);

	// special cases
	if (branchingPointIndex.first > -1) leftBranchingPointIndex = left[branchingPointIndex.first];
	if (branchingPointIndex.second > -1) rightBranchingPointIndex = right[branchingPointIndex.second];
	if (left.size() == 0 && right.size() != 0) rightBranchingPointIndex = right[0];   // take the first element so that there isn't a big gap
	if (right.size() == 0 && left.size() != 0) leftBranchingPointIndex = left[left.size() - 1];
	if (left.size() == 1) leftBranchingPointIndex = left[0];
	if (right.size() == 1) rightBranchingPointIndex = right[0];

	// interpolate points in between
	if (leftSide) {  // new branch added on the left
		int rightMostIndex = rebind.back();
		for (int i = rebind.front(); i < leftBranchingPointIndex; i++) {
			bindings[i].parentNode = newNode->parent->parent;
			bindings[i].childNode = newNode->parent;
			bindings[i].blending = glm::clamp(((float)(i) / (leftBranchingPointIndex + 1)), 0.f, 1.f);
			bindings[i].t = glm::clamp(((float)(i) / (leftBranchingPointIndex + 1)), 0.f, 1.f);
			if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && newNode->parent->parent->parent != NULL) {  // snap if t is close to 0
				bindings[i].parentNode = newNode->parent->parent->parent;
				bindings[i].childNode = newNode->parent->parent;
				bindings[i].t = 1;
				bindings[i].blending = 1;
			}
			bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
			bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
		}
		if (leftBranchingPointIndex != -1) {
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {
				bindings[i].parentNode = newNode->parent;
				bindings[i].childNode = newNode;
				bindings[i].blending = glm::clamp((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex + 1), 0.f, 1.f);
				bindings[i].t = glm::clamp((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex + 1), 0.f, 1.f);
				if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && newNode->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent;
					bindings[i].childNode = newNode->parent;
					bindings[i].t = 1;
					bindings[i].blending = 1;
				}
				bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
				bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
				bindings[i].newBranchBinding = true;
			}
		}
		for (int i = index + 1; i < rightBranchingPointIndex; i++) {
			bindings[i].parentNode = newNode->parent;
			bindings[i].childNode = newNode;
			bindings[i].blending = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index + 1)), 0.f, 1.f);
			bindings[i].t = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index + 1)), 0.f, 1.f);
			if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && newNode->parent->parent != NULL) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].t = 1;
				bindings[i].blending = 1;
			}
			bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
			bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
			bindings[i].newBranchBinding = true;
		}
		if (rightBranchingPointIndex != -1) {
			for (int i = rightBranchingPointIndex + 1; i <= rebind.back(); i++) {
				bindings[i].parentNode = bindings[rightMostIndex].childNode->parent;
				bindings[i].childNode = bindings[rightMostIndex].childNode;
				bindings[i].blending = glm::clamp((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex + 1), 0.f, 1.f);
				bindings[i].t = glm::clamp((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex + 1), 0.f, 1.f);
				if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && bindings[rightMostIndex].parentNode->parent != NULL) {
					bindings[i].parentNode = bindings[rightMostIndex].parentNode->parent;
					bindings[i].childNode = bindings[rightMostIndex].parentNode;
					bindings[i].t = 1;
					bindings[i].blending = 1;
				}
				bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
				bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
			}
		}
	}
	if (rightSide) {    // new branch added on the right
		int leftMostIndex = rebind.front();
		for (int i = rebind.front(); i < leftBranchingPointIndex; i++) {
			bindings[i].parentNode = bindings[leftMostIndex].parentNode;
			bindings[i].childNode = bindings[leftMostIndex].childNode;
			bindings[i].blending = glm::clamp(1 - ((float)(i - rebind.front() + 1) / (leftBranchingPointIndex - rebind.front() + 1)), 0.f, 1.f);
			bindings[i].t = glm::clamp(1 - ((float)(i - rebind.front() + 1) / (leftBranchingPointIndex - rebind.front() + 1)), 0.f, 1.f);
			if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && bindings[leftMostIndex].parentNode->parent != NULL) {
				bindings[i].parentNode = bindings[leftMostIndex].parentNode->parent;
				bindings[i].childNode = bindings[leftMostIndex].parentNode;
				bindings[i].t = 1;
				bindings[i].blending = 1;
			}
			bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
			bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
		}
		if (leftBranchingPointIndex != -1) {
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {
				bindings[i].parentNode = newNode->parent;
				bindings[i].childNode = newNode;
				bindings[i].blending = glm::clamp(((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex)), 0.f, 1.f);
				bindings[i].t = glm::clamp(((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex)), 0.f, 1.f);
				if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && newNode->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent;
					bindings[i].childNode = newNode->parent;
					bindings[i].t = 1;
					bindings[i].blending = 1;
				}
				bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
				bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
				bindings[i].newBranchBinding = true;
			}
		}
		for (int i = index + 1; i < rightBranchingPointIndex; i++) {
			bindings[i].parentNode = newNode->parent;
			bindings[i].childNode = newNode;
			bindings[i].blending = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index)), 0.f, 1.f);
			bindings[i].t = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index)), 0.f, 1.f);
			if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05) && newNode->parent->parent != NULL) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].t = 1;
				bindings[i].blending = 1;
			}
			bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
			bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
			bindings[i].newBranchBinding = true;
		}
		if (rightBranchingPointIndex != -1) {
			for (int i = rightBranchingPointIndex + 1; i <= rebind.back(); i++) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].blending = glm::clamp(1 - ((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex + 1)), 0.f, 1.f);
				bindings[i].t = glm::clamp(1 - ((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex + 1)), 0.f, 1.f);
				if ((bindings[i].t < 0.05 || bindings[i].blending < 0.05f) && newNode->parent->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent->parent;
					bindings[i].childNode = newNode->parent->parent;
					bindings[i].t = 1;
					bindings[i].blending = 1;
				}
				bindings[i].closestPoint = calculateClosestPoint(bindings[i]);
				bindings[i].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[i]));
			}
		}
	}
	// branching point rebinding
	if (leftBranchingPointIndex != -1) {
		bindings[leftBranchingPointIndex].parentNode = newNode->parent->parent;
		bindings[leftBranchingPointIndex].childNode = newNode->parent;
		bindings[leftBranchingPointIndex].blending = 1;
		bindings[leftBranchingPointIndex].t = 1;
		bindings[leftBranchingPointIndex].closestPoint = calculateClosestPoint(bindings[leftBranchingPointIndex]);
		bindings[leftBranchingPointIndex].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[leftBranchingPointIndex]));
		bindings[leftBranchingPointIndex].newBranchBinding = true;
	}
	if (rightBranchingPointIndex != -1) {
		bindings[rightBranchingPointIndex].parentNode = newNode->parent->parent;
		bindings[rightBranchingPointIndex].childNode = newNode->parent;
		bindings[rightBranchingPointIndex].blending = 1;
		bindings[rightBranchingPointIndex].t = 1;
		bindings[rightBranchingPointIndex].closestPoint = calculateClosestPoint(bindings[rightBranchingPointIndex]);
		bindings[rightBranchingPointIndex].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[rightBranchingPointIndex]));
		bindings[rightBranchingPointIndex].newBranchBinding = true;
	}
	// direct contour rebinding
	if (index != -1) {
		bindings[index].parentNode = newNode->parent;
		bindings[index].childNode = newNode;
		bindings[index].blending = 1;
		bindings[index].t = 1;
		bindings[index].closestPoint = calculateClosestPoint(bindings[index]);
		bindings[index].previousAnimateInverse = glm::inverse(calculateAnimationMatrix(bindings[index]));
		bindings[index].newBranchBinding = true;
	}

	// if leftBranchingPointIndex or rightBranchingPointIndex is -1, insert a new point
	if (leftBranchingPointIndex == -1 && leftSide) {
		float blending = 1;
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, newNode->parent));
		glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, newNode->parent);
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index - 1].parentNode->globalTransformation[3] - bindings[index - 1].childNode->globalTransformation[3]) / glm::length(bindings[index - 1].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index - 1].contourPoint + (1 - branchT) * bindings[index].contourPoint;
		bindings.insert(bindings.begin() + index, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true, blending });
	}
	if (leftBranchingPointIndex == -1 && rightSide) {
		float blending = 1;
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, newNode->parent));
		glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, newNode->parent);
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]) / glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index - 1].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index].contourPoint + (1 - branchT) * bindings[index - 1].contourPoint;
		bindings.insert(bindings.begin() + index, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true, blending });
	}
	if (rightBranchingPointIndex == -1 && leftSide) {
		float blending = 1;
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, newNode->parent));
		glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, newNode->parent);
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]) / glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index + 1].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index].contourPoint + (1 - branchT) * bindings[index + 1].contourPoint;
		bindings.insert(bindings.begin() + index + 1, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true, blending });
	}
	if (rightBranchingPointIndex == -1 && rightSide) {
		float blending = 1;
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, newNode->parent));
		glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, newNode->parent);
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index + 1].parentNode->globalTransformation[3] - bindings[index + 1].childNode->globalTransformation[3]) / glm::length(bindings[index + 1].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index + 1].contourPoint + (1 - branchT) * bindings[index].contourPoint;
		bindings.insert(bindings.begin() + index + 1, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true, blending });
	}

	// reset new branch
	newNode->addBranch = false;
}
// -----------------------------------

// destructor
void SceneNode::deleteSceneGraph(SceneNode* node) {
	if (!node) return;
	for (SceneNode* child : node->children) {
		deleteSceneGraph(child);
	}
	delete node;
}

// get all leaf nodes
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

	glm::vec3 leftOffset = rootPos - glm::vec3(0.1f, 0.25f, 0.0f);
	glm::vec3 rightOffset = rootPos + glm::vec3(0.1f, -0.25f, 0.0f);

	controlPoints.push_back(leftOffset);

	// leaf
	std::vector<SceneNode*> leaves;
	getLeafNodes(root, leaves);

	for (SceneNode* leaf : leaves) {
		glm::vec3 leafPos = leaf->globalTransformation[3];
		glm::vec3 leafParentPos = leaf->parent->globalTransformation[3];
		glm::vec3 dir = glm::normalize(leafPos - leafParentPos);
		//glm::vec3 offsetPos = leafPos + dir * 0.15f;
		glm::vec3 offsetPos = leafPos;
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

std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> SceneNode::contourCatmullRomGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches) {
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

			//if (j == pointsPerSegment / 2) {
			//	float minDist = FLT_MAX;
			//	for (auto& [parent, child, index] : branches) {
			//		glm::vec3 P = parent->globalTransformation[3];
			//		glm::vec3 Q = child->globalTransformation[3];
			//		glm::vec3 closest = intersectionPoint(P, Q, pt);
			//		float dist = glm::length(closest - pt);
			//		if (dist < minDist) {
			//			minDist = dist;
			//			segment = { parent, child };
			//		}
			//	}
			//}
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

std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> SceneNode::contourLinearGrouped(
	std::vector<glm::vec3> controlPoints,
	int pointsPerSegment,
	std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches
) {
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContourPoints;
	std::pair<SceneNode*, SceneNode*> segment;

	int branchCounter = 0;
	int branchIndex = 0;
	for (size_t i = 0; i < controlPoints.size() - 1; i++) {
		std::vector<glm::vec3> segmentPoints;

		for (int j = 0; j < pointsPerSegment; j++) {
			float t = float(j) / pointsPerSegment;

			glm::vec3 pt = (1.0f - t) * controlPoints[i] + t * controlPoints[i + 1];
			//if (t != 1) pt.x += pt.x;
			segmentPoints.push_back(pt);
			segment = { std::get<0>(branches[branchIndex]), std::get<1>(branches[branchIndex]) };

			// assumption is that every branch will have 2 groups of points binded to it

			//if (j == pointsPerSegment / 2) {
			//	float minDist = FLT_MAX;
			//	for (auto& [parent, child, index] : branches) {
			//		glm::vec3 P = parent->globalTransformation[3];
			//		glm::vec3 Q = child->globalTransformation[3];

			//		glm::vec3 closest = intersectionPoint(P, Q, pt);
			//		float dist = glm::length(closest - pt);

			//		if (dist < minDist) {
			//			minDist = dist;
			//			segment = { parent, child };
			//		}
			//	}
			//}
		}
		groupedContourPoints.push_back({ segmentPoints, segment });
		branchCounter++;
		if (branchCounter > 1) {
			branchCounter = 0;
			branchIndex++;
		}

		// last contour point
		if (i == controlPoints.size() - 2) groupedContourPoints.push_back({ {controlPoints[i + 1]}, segment });
	}

	return groupedContourPoints;
}

std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> SceneNode::contourQuadraticGrouped(
	std::vector<glm::vec3> controlPoints,
	int pointsPerSegment,
	std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches
) {
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContourPoints;
	std::pair<SceneNode*, SceneNode*> segment;

	int branchCounter = 0;
	int branchIndex = 0;
	for (size_t i = 0; i < controlPoints.size() - 2; i++) {
		std::vector<glm::vec3> segmentPoints;

		glm::vec3 P0 = controlPoints[i];
		glm::vec3 P1 = controlPoints[i + 1];
		glm::vec3 P2 = controlPoints[i + 2];

		for (int j = 0; j <= pointsPerSegment; j++) {
			float t = float(j) / pointsPerSegment;
			glm::vec3 pt = (1.0f - t) * (1.0f - t) * P0
				+ 2.0f * (1.0f - t) * t * P1
				+ t * t * P2;

			segmentPoints.push_back(pt);
			segment = { std::get<0>(branches[branchIndex]), std::get<1>(branches[branchIndex]) };
		}

		groupedContourPoints.push_back({ segmentPoints, segment });
		branchCounter++;
		if (branchCounter > 1) {
			branchCounter = 0;
			branchIndex++;
		}

		if (i == controlPoints.size() - 3) groupedContourPoints.push_back({ { P2 }, segment });
	}

	return groupedContourPoints;
}

// guarantees even interpolation along the branch for the contour points (key in interpolative skinning)
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
			else t = (j / ((float)contourPoints[i].first.size()));   // t is number of points based
			if (contourPoints[i].first.size() == 1) t = 0;
			//t=fminf(fmaxf((contourPoints[i].first[j].y- P.y)/(Q.y-P.y),0.0),1.0);
			glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, child);

			// blending transformation
			if (i % 2 == 1) blending = 1 - (j / ((float)contourPoints[i].first.size()));
			else blending = (j / ((float)contourPoints[i].first.size()));
			if (contourPoints[i].first.size() == 1) blending = 0;
			//blending=fminf(fmaxf((contourPoints[i].first[j].y- P.y)/(Q.y-P.y),0.0),1.0);
			glm::mat4 previousAnimateInverse = glm::inverse(calculateAnimationMatrixForNewPoint(blending, child));

			glm::vec3 bindingdirection = (contourPoints[i].first[j] - closestPoint) / 2.f;
			bestBinding = { parent, child, contourPoints[i].first[j], t, closestPoint, previousAnimateInverse, false, blending };
			bindings.push_back(bestBinding);
		}
	}

	return bindings;
}

// just directly binding to closestPoint
std::vector<ContourBinding> SceneNode::bindContourToBranches(const std::vector<glm::vec3>& contourPoints, SceneNode* root, std::vector<std::pair<SceneNode*, SceneNode*>>& segments) {
	std::vector<ContourBinding> bindings;
	glm::vec3 rootPos = root->globalTransformation[3];

	for (const glm::vec3& contourPoint : contourPoints) {
		float minDist = FLT_MAX;
		ContourBinding bestBinding;

		glm::vec3 rootToContour = glm::normalize(contourPoint - rootPos);
		for (auto& [parent, child] : segments) {
			glm::vec3 P = parent->globalTransformation[3];
			glm::vec3 Q = child->globalTransformation[3];

			glm::vec3 closest = intersectionPoint(P, Q, contourPoint);
			float t = glm::dot(Q - P, contourPoint - P) / glm::dot(Q - P, Q - P);  // t is simply the projection of contour point on the branch
			t = glm::clamp(t, 0.0f, 1.0f);
			float dist = glm::length(closest - contourPoint);

			if (dist < minDist) {
				minDist = dist;
				bestBinding = { parent, child, contourPoint, t, closest, glm::inverse(calculateAnimationMatrixForNewPoint(t, child)), false, t };

			}
		}

		bindings.push_back(bestBinding);
	}

	return bindings;
}


// extract parent child pair (endpoints of the branches)
void SceneNode::getBranches(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments) {
	for (SceneNode* child : node->children) {
		segments.push_back({ node, child });
		getBranches(child, segments);
	}
}

void SceneNode::printBranches(SceneNode* node, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments, int& i) {
	std::cout << glm::vec3(node->globalTransformation[3]) << std::endl;
	std::cout << node->axisID << std::endl;
	for (SceneNode* child : node->children) {
		i++;
		printBranches(child, segments, i);
	}
	//for (SceneNode* child : node->children) {
	//	segments.push_back({ node, child, i });
	//	std::cout << "parent: " << glm::vec3(node->globalTransformation[3]) << std::endl;
	//	std::cout << "child: " << glm::vec3(child->globalTransformation[3]) << std::endl;
	//	i++;
	//	printBranches(child, segments, i);
	//}
}

void SceneNode::printTree(SceneNode* node, int depth) {
	std::string indent(depth * 2, ' ');
	std::cout << indent << "node: " << glm::vec3(node->globalTransformation[3]) << std::endl;
	for (SceneNode* child : node->children) {
		std::cout << indent << "  child: " << glm::vec3(child->globalTransformation[3]) << std::endl;
	}
	std::cout << std::endl;
	for (SceneNode* child : node->children) {
		printTree(child, depth + 1);
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

	//std::cout << "Node: " << glm::vec3(node->globalTransformation[3])
	//	<< " AxisID: " << node->axisID << std::endl;

	for (auto* child : node->children)
	{
		maxID = std::max(maxID, getMaxID(child));
	}

	return maxID;
}

// returns vector of all nodes
void collectNodes(SceneNode* node, std::vector<SceneNode*>& nodes)
{
	if (!node)
		return;

	nodes.push_back(node);

	for (SceneNode* child : node->children)
	{
		collectNodes(child, nodes);
	}
}

std::vector<SceneNode*> getAllNodes(SceneNode* root)
{
	std::vector<SceneNode*> nodes;
	collectNodes(root, nodes);
	return nodes;
}

// root -> leaf branch path
void SceneNode::buildBranches(
	SceneNode* node,
	std::vector<SceneNode*>& currentPath,
	std::vector<Branch>& branches
) {
	if (!node) return;

	// add current node to path
	currentPath.push_back(node);

	// if leaf → store branch
	if (node->children.empty()) {
		Branch b;
		b.root = currentPath.front();
		b.leaf = node;
		b.nodes = currentPath;
		branches.push_back(b);

		currentPath.pop_back();
		return;
	}

	// recurse children
	for (SceneNode* child : node->children) {
		buildBranches(child, currentPath, branches);
	}

	// backtrack
	currentPath.pop_back();
}

std::vector<Branch> SceneNode::generateAllBranches(SceneNode* root) {
	std::vector<Branch> branches;
	std::vector<SceneNode*> currentPath;

	buildBranches(root, currentPath, branches);

	return branches;
}
// interpolate branches (?)
void SceneNode::interpolateBranchTransforms(std::vector<std::pair<SceneNode*, SceneNode*>>& pair, std::vector<CPU_Geometry>& outGeometry) {
	for (auto& [parent, child] : pair) {
		glm::mat4 T1 = parent->globalTransformation;
		glm::mat4 T2 = child->globalTransformation;
		CPU_Geometry geom;

		for (int i = 0; i <= 7; i++) {
			float t = static_cast<float>(i) / 7.0f;

			glm::mat4 animatedMat =
				t * child->globalTransformation * child->restPoseInverse +
				(1 - t) * parent->globalTransformation * parent->restPoseInverse;

			glm::vec3 pos = t * child->restPose[3] + (1 - t) * parent->restPose[3];

			geom.verts.push_back(glm::vec3(animatedMat * glm::vec4(pos, 1.0f)));
			geom.cols.push_back(glm::vec3(0.f, 0.8f, 0.f));
		}

		outGeometry.push_back(geom);
	}
}

// add contour points when branch is split
// contour points binded to the new node (split point)
std::vector<ContourBinding> SceneNode::addNewContourToBindToNewBranchNode(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs) {
	std::vector<ContourBinding> newBindingSet;
	for (int i = 0; i < bindings.size() - 1; i++) {
		// two consecutive contour points belong to different branches (so a newly inserted branch node exists)
		// assuming that there is only one newly added branch node between the existing ones

		// left side
		if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].childNode == bindings[i + 1].parentNode && bindings[i].t != 1) {
			float blending = 1;
			float t = 1;
			glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, bindings[i].childNode));
			glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, bindings[i].childNode);
			// find the t for the branch node
			float branchT = glm::length(bindings[i].closestPoint - glm::vec3(bindings[i].childNode->globalTransformation[3])) / glm::length(bindings[i].closestPoint - bindings[i + 1].closestPoint);
			glm::vec3 newPoint = branchT * bindings[i + 1].contourPoint + (1 - branchT) * bindings[i].contourPoint;
			//float branchT = glm::length(bindings[i].parentNode->globalTransformation[3] - bindings[i].childNode->globalTransformation[3]) / glm::length(bindings[i].parentNode->globalTransformation[3] - bindings[i + 1].childNode->globalTransformation[3]);
			//glm::vec3 newPoint = branchT * bindings[i].contourPoint + (1 - branchT) * bindings[i + 1].contourPoint;
			newBindingSet.push_back(bindings[i]);
			newBindingSet.push_back({ bindings[i].parentNode, bindings[i].childNode, newPoint, t, closestPoint, previousiInverseAnimationMat, bindings[i].newBranchBinding, blending });
		}
		// right side
		else if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].parentNode == bindings[i + 1].childNode && bindings[i + 1].t != 1) {
			float t = 1;
			float blending = 1;
			glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, bindings[i + 1].childNode));
			glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, bindings[i + 1].childNode);
			// find the t for the branch node
			float branchT = glm::length(bindings[i + 1].closestPoint - glm::vec3(bindings[i + 1].childNode->globalTransformation[3])) / glm::length(bindings[i + 1].closestPoint - bindings[i].closestPoint);
			glm::vec3 newPoint = branchT * bindings[i].contourPoint + (1 - branchT) * bindings[i + 1].contourPoint;
			/*float branchT = glm::length(bindings[i + 1].parentNode->globalTransformation[3] - bindings[i + 1].childNode->globalTransformation[3]) / glm::length(bindings[i + 1].parentNode->globalTransformation[3] - bindings[i].childNode->globalTransformation[3]);
			glm::vec3 newPoint = branchT * bindings[i + 1].contourPoint + (1 - branchT) * bindings[i].contourPoint;*/
			newBindingSet.push_back(bindings[i]);
			newBindingSet.push_back({ bindings[i + 1].parentNode, bindings[i + 1].childNode, newPoint, t, closestPoint, previousiInverseAnimationMat, bindings[i + 1].newBranchBinding, blending });
		}
		else {
			newBindingSet.push_back(bindings[i]);
		}
	}
	newBindingSet.push_back(bindings[bindings.size() - 1]);
	return newBindingSet;
}

std::vector<ContourBinding> SceneNode::snapContourPoints(std::vector<ContourBinding>& bindings) {
	std::vector<ContourBinding> newBindingSet;
	for (int i = 0; i < bindings.size() - 1; i++) {
		// only snap if the points are not binded to branching point
		if (glm::length(bindings[i].closestPoint - bindings[i + 1].closestPoint) < 1e-04f) {
			newBindingSet.push_back(bindings[i]);
			i++;
		}
		else {
			newBindingSet.push_back(bindings[i]);
		}
	}
	newBindingSet.push_back(bindings[bindings.size() - 1]);
	return newBindingSet;
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

// add contour points and bind if distance between two contour points exceed a threshold
std::vector<ContourBinding> SceneNode::addContourPoints(std::vector<ContourBinding>& bindings) {
	std::vector<ContourBinding> newBindingSet;
	// arbitrary threshold
	float threshold = 0.05f;
	for (int i = 0; i < bindings.size() - 1; i++) {
		float distance = glm::length(bindings[i + 1].contourPoint - bindings[i].contourPoint);
		if (distance > threshold) {
			glm::vec3 newPoint = glm::mix(bindings[i].contourPoint, bindings[i + 1].contourPoint, 0.5f);
			// special case: contour points belong to different branch
			if (bindings[i].childNode != bindings[i + 1].childNode) {
				// find younger branch to add to that branch
				ContourBinding neighbor = (getDeeperNode(bindings[i].childNode, bindings[i + 1].childNode) == bindings[i].childNode) ? bindings[i] : bindings[i + 1];
				glm::vec3 neighborParent = neighbor.parentNode->globalTransformation[3];
				glm::vec3 neighborChild = neighbor.childNode->globalTransformation[3];
				float blending = neighbor.blending / 2.f;
				float t = neighbor.t / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, neighbor.childNode));
				glm::vec3 closestPoint = calculateClosestPointForNewPoint(t, neighbor.childNode);
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ neighbor.parentNode, neighbor.childNode, newPoint, t, glm::mix(bindings[i].closestPoint, bindings[i + 1].closestPoint, 0.5f), previousiInverseAnimationMat, bindings[i].newBranchBinding, blending });
			}
			else {
				float blending = (bindings[i].blending + bindings[i + 1].blending) / 2.f;
				float t = (bindings[i].t + bindings[i + 1].t) / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(calculateAnimationMatrixForNewPoint(blending, bindings[i].childNode));
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ bindings[i].parentNode, bindings[i].childNode, newPoint, t, glm::mix(bindings[i].closestPoint, bindings[i + 1].closestPoint, 0.5f), previousiInverseAnimationMat, bindings[i].newBranchBinding, blending });
			}
		}
		else {
			newBindingSet.push_back(bindings[i]);
		}
	}
	newBindingSet.push_back(bindings[bindings.size() - 1]);

	return newBindingSet;
}

void SceneNode::calculateNormalDirection(std::vector<ContourBinding>& bindings) {
	for (int i = 1; i < bindings.size() - 1; i++) {
		glm::vec3 firstDirection = glm::normalize(bindings[i].contourPoint - bindings[i - 1].contourPoint);
		glm::vec3 secondDirection = glm::normalize(bindings[i + 1].contourPoint - bindings[i].contourPoint);
		glm::vec3 average = glm::vec3((firstDirection.x + secondDirection.x) / 2.f, (firstDirection.y + secondDirection.y) / 2.f, (firstDirection.z + secondDirection.z) / 2.f);   // take average to get tangent vector 
		bindings[i].normalDirection = glm::vec3(-average.y, average.x, average.z);  // rotate by 90 degrees counterclockwise in 2d
	}
}

// animate contour points for growth 
void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings, float deltaTime) {
	// first, calculate the delta transformation matrices for all points, store in an array
	// then blend the matrices
	// then apply

	// petiole: limit the growth for the first and last few points
	for (int i = 0; i < bindings.size(); i++) {
		glm::mat4 animatedPosMat;
		animatedPosMat = calculateAnimationMatrix(bindings[i]);
		//printMat4(animatedPosMat);
		//glm::vec3 bindingPosition = bindings[i].closestPoint;   // translating by -bindingPosition moves it back to local space
		bindings[i].contourPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].contourPoint, 1.0f);

		//// growth (translation) in the normal direction
		//if (true) {
		//	bindings[i].contourPoint.x += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.x;
		//	bindings[i].contourPoint.y += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.y;
		//}

		// transformation applied in local coordinate frame
		//binding.contourPoint = glm::translate(bindingPosition) * animatedPosMat * binding.previousAnimateInverse * glm::translate(-bindingPosition) * glm::vec4(binding.contourPoint, 1.0f);
		bindings[i].closestPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].closestPoint, 1.0f);
		bindings[i].previousAnimateInverse = glm::inverse(animatedPosMat);
		//binding.closestPoint = calculateClosestPoint(binding);

		// update t value to reflect the non-linear growth
		bindings[i].t = (glm::length(bindings[i].closestPoint - glm::vec3(bindings[i].parentNode->globalTransformation[3])) / glm::length(bindings[i].childNode->globalTransformation[3] - bindings[i].parentNode->globalTransformation[3]));

	}
}

//// relative transformation between frame (global frame) -> blending transformations for smoother transition
//void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings, float deltaTime) {
//	// first, calculate the delta transformation matrices for all points, store in an array
//	// then blend the matrices
//	// then apply
//
//	std::vector<glm::mat4> deltaTransformations;
//	std::vector<glm::mat4> blendedTransformations;
//	std::vector<int> category;
//	int cat = 0;
//	float prevT = -1.0f;
//	bool increasing = true;
//
//	// first calculate the delta transformations
//	for (int i = 0; i < bindings.size(); i++) {
//		glm::mat4 animatedPosMat;
//		animatedPosMat = calculateAnimationMatrix(bindings[i]);
//		deltaTransformations.push_back(animatedPosMat * bindings[i].previousAnimateInverse);
//
//		// transformation applied in local coordinate frame
//		//binding.contourPoint = glm::translate(bindingPosition) * animatedPosMat * binding.previousAnimateInverse * glm::translate(-bindingPosition) * glm::vec4(binding.contourPoint, 1.0f);
//		//bindings[i].closestPoint = animatedPosMat * bindings[i].previousAnimateInverse * glm::vec4(bindings[i].closestPoint, 1.0f);
//		bindings[i].previousAnimateInverse = glm::inverse(animatedPosMat);
//		//bindings[i].t = (glm::length(bindings[i].closestPoint - glm::vec3(bindings[i].parentNode->globalTransformation[3])) / glm::length(bindings[i].childNode->globalTransformation[3] - bindings[i].parentNode->globalTransformation[3]));
//
//		//// based on region
//		//if (prevT >= 0.0f) {
//		//	bool isIncreasing = bindings[i].t > prevT;
//		//	if (isIncreasing != increasing) {
//		//		cat++; // direction changed, move to new category
//		//		increasing = isIncreasing;
//		//	}
//		//}
//		//prevT = bindings[i].t;
//		//category.push_back(cat);
//
//		//// blending only near indentation
//		//if (bindings[i].t < 0.5f) category.push_back(cat);
//		//else {
//		//	cat++;
//		//	category.push_back(cat);
//		//}
//	}
//
//	// now blend the delta transformations
//	// using distance (all points within a distance specified)
//	float threshold = 0.5f;
//	std::vector<float> arcLengths(bindings.size(), 0.0f);
//
//	for (int i = 1; i < bindings.size(); i++) {
//		arcLengths[i] = arcLengths[i - 1] + glm::distance(bindings[i].contourPoint, bindings[i - 1].contourPoint);
//	}
//
//	// set region and blend
//	std::vector<float> nearBranchingPoint;  // is the current contour point close to the branching point?
//	std::vector<float> weights;				// weight to blend
//	for (int i = 0; i < bindings.size(); i++) {
//		float length = glm::length(bindings[i].parentNode->globalTransformation[3] - glm::vec4(bindings[i].contourPoint, 0.f));
//		if (bindings[i].t <= threshold) {
//			nearBranchingPoint.push_back(length);
//			weights.push_back(powf(bindings[i].t / threshold,2));   // t * 2 to make it 1 (normalize), since we are taking 0.5
//			// taking the square helps, because the derivative at 0 is continuous
//		}
//		else {
//			nearBranchingPoint.push_back(0.0f);
//			weights.push_back(1.f);
//		}
//	}
//
//	for (int i = 0; i < deltaTransformations.size(); i++) {
//		// to blend (if it is close to branching point)
//		if (nearBranchingPoint[i] > 0.0f) {    
//			float centerArc = arcLengths[i];  // current point is the center
//			glm::mat4 sum(0.0f);
//			int count = 0;
//
//			// blend
//			for (int j = 1; j < deltaTransformations.size() - 1; j++) {
//				float arcDist = std::abs(arcLengths[j] - centerArc);
//				if (arcDist <= threshold ) {  // decrease the blending region as you move further away from the branching point
//					// using the fact that t value increases as you move further away from the binding point
//					sum += deltaTransformations[j];
//					count++;
//				}
//			}
//
//			// check
//			if (count > 0) {
//				auto averaged_delta = sum / (float)count;
//				auto weighted_average = averaged_delta * (float)(1. - weights[i]) + deltaTransformations[i]* weights[i];   // point closest to branching point has weight 0, so need to 1 - weight
//				//auto weighted_average = averaged_delta;
//				blendedTransformations.push_back(weighted_average);
//			}
//			else {
//				blendedTransformations.push_back(deltaTransformations[i]);
//			}
//		}
//		// not to blend
//		else {
//			blendedTransformations.push_back(deltaTransformations[i]);
//		}
//	}
//
//	//// using fixed number of points left and right
//	//for (int i = 0; i < deltaTransformations.size(); i++) {
//	//	int count = 0;
//	//	if (i < 10 || i > deltaTransformations.size() - 11) {
//	//		blendedTransformations.push_back(deltaTransformations[i]);
//	//	}
//	//	else {
//	//		glm::mat4 sum(0.0f);
//	//		for (int offset = -10; offset <= 10; ++offset) {
//	//			sum += deltaTransformations[i + offset];
//	//			count++;
//	//		}
//	//		blendedTransformations.push_back(sum / static_cast<float>(count)); 
//	//	}
//	//}
//
//	// apply the transformations
//	for (int i = 0; i < blendedTransformations.size(); i++) {
//		bindings[i].contourPoint = blendedTransformations[i] * glm::vec4(bindings[i].contourPoint, 1.0f);
//		bindings[i].closestPoint = deltaTransformations[i] * glm::vec4(bindings[i].closestPoint, 1.0f);
//		//bindings[i].previousAnimateInverse = bindings[i].previousAnimateInverse * glm::inverse(blendedTransformations[i]);
//		//bindings[i].t = (glm::length(bindings[i].closestPoint - glm::vec3(bindings[i].parentNode->globalTransformation[3])) / glm::length(bindings[i].childNode->globalTransformation[3] - bindings[i].parentNode->globalTransformation[3]));
//
//		//// growth (translation) in the normal direction
//		//if (true) {
//		//	bindings[i].contourPoint.x += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.x;
//		//	bindings[i].contourPoint.y += deltaTime * bindings[i].normalFactor * bindings[i].normalDirection.y;
//		//}
//	}
//}



