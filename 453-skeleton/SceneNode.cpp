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

// just a helper function to print the matrices for debugging purposes
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

// Scene Graph Structure
SceneNode::SceneNode() : localTranslation(1.0f), localRotation(1.0f, 0.0f, 0.0f, 0.0f), localScaling(1.0f), parent(nullptr) {}

// function to add children to current node
void SceneNode::addChild(SceneNode* child) {
	child->parent = this;
	children.push_back(child);
}

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

// extract the local matrices per edge
std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float>> SceneNode::extractEdgeTransforms(const std::string& filename) {
	std::ifstream in(filename);
	if (!in.is_open()) {
		std::cerr << "Failed to open file\n";
		return {};
	}

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float>> edges;
	std::string line;
	std::regex edgeRegex(R"#(Edge\s+(\d+)\s+->\s+(\d+))#");
	std::smatch match;

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

			edges.emplace_back(parent, child, rotation, scaling, translation, scalingFactor, rotationDirection, rotationAngle);
		}
	}

	return edges;
}

std::vector<std::vector<int>> SceneNode::buildChildrenList(
	const std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float>>& edges
) {
	// maximum node index
	int maxIndex = 0;
	for (const auto& [parent, child, rot, scale, trans, scaleF, rotationD, rotationA] : edges) {
		maxIndex = std::max({ maxIndex, parent, child });
	}

	std::vector<std::vector<int>> childrenList(maxIndex + 1);

	for (const auto& [parent, child, rot, scale, trans, scaleF, rotationD, rotationA] : edges) {
		childrenList[parent].push_back(child);

	}

	return childrenList;
}


SceneNode* SceneNode::createBranchingStructure(
	int nodeIndex, std::vector<std::vector<int>> parentChildPairs, std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float>> transformations) {
	// create node
	SceneNode* node = new SceneNode();
	node->localTranslation = glm::mat4(1.0f);
	node->localRotation = glm::quat(1.0f, 0.f, 0.f, 0.f);
	node->localScaling = glm::mat4(1.0f);
	node->S = 0.f;
	node->animationDirection = 0;
	node->rotationAngle = 0.f;

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

		node->addChild(childNode);
	}

	return node;
}

bool animated = false;
// animation
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
	animationScaling = (1 + deltaTime * S) * animationScaling;
	// uniform scaling
	animateScaling = glm::scale(glm::mat4(1.0f), glm::vec3(animationScaling));
	// non-uniform scaling
	//animateScaling = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f, animationScaling, 1.0f));

	for (SceneNode* child : children) {
		child->animate(deltaTime);
	}
}

// compute global transformation for all nodes recursively
// to get the final position and draw
void SceneNode::updateBranch(const glm::mat4& parentTransform, const glm::mat4& parentRestInverse, const glm::mat4& parentRest, CPU_Geometry& outGeometry) {
	// convert rotation quaternion back to matrix form
	glm::mat4 animateRotationMatrix = glm::toMat4(animateRotation);
	glm::mat4 localRotationMatrix = glm::toMat4(localRotation);

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

	////globalTransformation = parentTransform * localRotationMatrix *animateScaling * localScaling *  localTranslation;
	//////we don't want scaling to affect the child
	////glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	////temp2[0][0] = 1.0;
	////temp2[1][1] = 1.0;
	////temp2[2][2] = 1.0;
	//glm::mat4 newParentTransform = parentTransform * animateRotationMatrix * localRotationMatrix * temp2;


	// removing scaling
	glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	temp2[0][0] = 1.0;
	temp2[1][1] = 1.0;
	temp2[2][2] = 1.0;
	globalTransformation = parentTransform * localRotationMatrix * animateScaling * localScaling * localTranslation;

	//globalTransformation = parentTransform * localRotationMatrix *animateScaling * localScaling *  localTranslation;
	////we don't want scaling to affect the child
	//glm::mat4 temp2 = animateScaling * localScaling * localTranslation;
	//temp2[0][0] = 1.0;
	//temp2[1][1] = 1.0;
	//temp2[2][2] = 1.0;
	glm::mat4 newParentTransform = parentTransform * localRotationMatrix * temp2;

	// global to local rest post matrix
	// need to apply parentRest outside because if not inverse will accumulate (inverse every call)
	restPoseInverse = glm::inverse(localScaling * localRotationMatrix * localTranslation) * parentRestInverse;
	// rest pose matrix
	restPose = parentRest * localScaling * localRotationMatrix * localTranslation;
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
		child->updateBranch(newParentTransform, restPoseInverse, restPose, outGeometry);
	}
}

void SceneNode::printStructure(SceneNode* node) {
	std::cout << glm::to_string(node->globalTransformation[3]) << std::endl;
	for (SceneNode* child : node->children) {
		printStructure(child);
	}
}

void SceneNode::printMatrix(SceneNode* node) {
	printMat4(node->globalTransformation);
	for (SceneNode* child : node->children) {
		printMatrix(child);
	}
}

bool SceneNode::divideBranch(SceneNode* node, float threshold) {
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
			glm::vec3 midPos = glm::mix(parentPos, childPos, 0.5f);

			// midNode inherits from child
			midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - parentPos)));
			//midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, ((glm::length(midPos - parentPos) / glm::length(childPos - parentPos)) * (child->localTranslation * glm::vec4(0, 0, 0, 1)).y),0));
			//midNode->localScaling = child->localScaling;
			midNode->localRotation = child->localRotation;
			midNode->S = child->S;
			midNode->rotationAngle = child->rotationAngle;
			midNode->animationDirection = child->animationDirection;
			midNode->animationAngle = child->animationAngle;
			midNode->animateRotation = child->animateRotation;
			midNode->animationScaling = 1;
			midNode->animateScaling = glm::mat4(1.0);

			// child set to identity
			child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(midPos - childPos)));
			//child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, ((glm::length(midPos - parentPos) / glm::length(childPos - parentPos)) * (child->localTranslation * glm::vec4(0, 0, 0, 1)).y), 0));
			child->localRotation = glm::mat4(1.f);
			child->rotationAngle = 0;
			child->animationDirection = 0;
			child->animationAngle = 0.f;
			child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
			child->animationScaling = 1;
			child->animateScaling = glm::mat4(1.0);

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
		divided = divided || divideBranch(child, threshold);
	}
	return divided;
}

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

std::tuple<SceneNode*, SceneNode*, float, glm::vec3> findClosestPointBranch(std::vector<std::pair<SceneNode*, SceneNode*>>& segments,ContourBinding* contour, bool& found) {
	SceneNode* p;
	SceneNode* c;
	float finalT;

	// only rebind if the point's branch has been divided
	if (contour->childNode->midBranch && contour->parentNode->midBranch) {
		// find the divided segments
		getBranchSegmentsFromBinding(contour->parentNode, contour->childNode, segments, found);
		// special case, needed for floating point error
		if (contour->t == 0) {
			for (const auto& [parent, child] : segments) {
				if (parent == contour->parentNode) {
					p = parent;
					c = child;
					finalT = contour->t;
				}
			}
		}
		else if (contour->t == 1) {
			for (const auto& [parent, child] : segments) {
				if (child == contour->childNode) {
					p = parent;
					c = child;
					finalT = contour->t;
				}
			}
		}
		else {
			for (const auto& [parent, child] : segments) {
				float distance = FLT_MAX;
				glm::vec3 p1 = glm::vec3(parent->globalTransformation[3]);
				glm::vec3 p2 = glm::vec3(child->globalTransformation[3]);

				glm::vec3 dir = p2 - p1;
				float t = glm::dot(contour->closestPoint - p1, dir) / glm::dot(dir, dir);

				if (t < -1e-4f || t > 1.0f + 1e-4f) continue;
				t = glm::clamp(t, 0.0f, 1.0f);

				// we are only considering the branches that are part of the original branch, so no need to project the point and compare distance
				// it is guaranteed to belong in only one of the branch segments
				p = parent;
				c = child;
				finalT = t;
			}
		}
	}
	// otherwise, keep it the same
	else {
		p = contour->parentNode;
		c = contour->childNode;
		finalT = contour->t;
	}

	return std::make_tuple(p, c, finalT, contour->closestPoint);
}

void SceneNode::rebindContour(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments, int& i, std::vector<ContourBinding>& bindings) {
	for (ContourBinding& binding : bindings) {
		// get the current closest point, find which branch to bind to
		bool found = false;
		std::tuple<SceneNode*, SceneNode*, float, glm::vec3> branch = findClosestPointBranch(segments, &binding, found);
		// guarantee t > 0
		if (std::get<2>(branch) == 0 && std::get<0>(branch)->parent != NULL) {
			binding.parentNode = std::get<0>(branch)->parent;
			binding.childNode = std::get<0>(branch);
			binding.t = 1;
			binding.previousAnimateInverse = glm::inverse(binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation);
			binding.closestPoint = binding.t * binding.childNode->globalTransformation[3] + (1 - binding.t) * binding.parentNode->globalTransformation[3];
			continue;
		}
		binding.parentNode = std::get<0>(branch);
		binding.childNode = std::get<1>(branch);
		binding.t = std::get<2>(branch);
		binding.previousAnimateInverse = glm::inverse(std::get<2>(branch) * std::get<1>(branch)->globalTransformation + (1 - std::get<2>(branch)) * std::get<0>(branch)->globalTransformation);
		binding.closestPoint = std::get<2>(branch) * std::get<1>(branch)->globalTransformation[3] + (1 - std::get<2>(branch)) * std::get<0>(branch)->globalTransformation[3];
	}
	// rebind is done, reset everything
	for (std::tuple<SceneNode*, SceneNode*> segment : segments) {
		std::get<0>(segment)->midBranch = false;
		std::get<1>(segment)->midBranch = false;
	}
}

// ADD NEW BRANCH
// probably need a better way to find the contour point to add a new branch to
ContourBinding* SceneNode::findContourPointToAddBranch(float height, SceneNode* root, std::vector<ContourBinding>& contourPoints) {
	ContourBinding* finalContour = nullptr;
	//std::vector<ContourBinding*> finalContours;
	float difference = FLT_MAX;

	// don't want to connect with the first and last 
	for (size_t i = 1; i < contourPoints.size() - 1; i++) {
		ContourBinding& contour = contourPoints[i];
		float dist = abs(glm::length(glm::vec3(root->globalTransformation[3]) - contour.contourPoint) - height);
		if (dist < difference) {
			difference = dist;
			finalContour = &contour;
		}
	}

	return finalContour; 
}

// no need to consider about visibility, just consider the simplest case (main axis)
ContourBinding findBestBinding(SceneNode* root, const glm::vec3& contourPoint) {
	ContourBinding bestBinding;
	float minTotalDistance = std::numeric_limits<float>::max();

	// just consider main axis (one)
	std::function<void(SceneNode*)> dfs = [&](SceneNode* node) {
		for (SceneNode* child : node->children) {
			glm::vec3 parentPos = glm::vec3(node->globalTransformation[3]);
			glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
			glm::vec3 dir = childPos - parentPos;

			const int steps = 50; 
			for (int i = 0; i <= steps; ++i) {
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
				if (totalDistance < minTotalDistance) {
					minTotalDistance = totalDistance;
					// special case for tip of the branch, to preserve the tip of the main axis
					if (t == 1) {
						t = 0.9f;
						projected = parentPos + t * dir;
					}
					// return best division
					bestBinding = {
						node,
						child,
						contourPoint,
						t,
						projected,
						glm::mat4(1.0f)
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

			// break in the closest point
			midNode->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			//midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(contour->closestPoint - parentPos)));
			midNode->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(pointToBreak.closestPoint - parentPos)));
			midNode->localRotation = child->localRotation;
			midNode->S = child->S;
			midNode->rotationAngle = child->rotationAngle;
			//float ratio = glm::length(contour->closestPoint - parentPos) / glm::length(childPos - parentPos);
			//midNode->animationDirection = child->animationDirection * ratio;                                   // otherwise, rotation is too slow (just controls left/right rotation) since animation rotation will decrease
			midNode->animationDirection = child->animationDirection;
			midNode->animationAngle = child->animationAngle;
			midNode->animateRotation = child->animateRotation;
			midNode->animationScaling = 1;
			midNode->animateScaling = glm::mat4(1.0);

			// child set to identity
			child->localTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.f, 1.f, 0.f));
			//child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(childPos - contour->closestPoint)));
			child->localScaling = glm::scale(glm::mat4(1.0f), glm::vec3(glm::length(childPos - pointToBreak.closestPoint)));
			child->localRotation = glm::mat4(1.f);
			child->rotationAngle = 0;
			//child->animationDirection = child->animationDirection / (1 - ratio);
			child->animationDirection = 0;
			child->animationAngle = 0.f;
			child->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
			child->animationScaling = 1;
			child->animateScaling = glm::mat4(1.0);

			midNode->addChild(child);
			midNode->parent = node;
			updatedChildren.push_back(midNode);
			// distinguish as new branch
			node->midBranch = true;
			midNode->midBranch = true;
			child->midBranch = true;
			// to rebind all contour points that are binded to these branches; need to exclusively include the contour's original branch because the new branch might be added to a different branch
			// so if we don't include the contour's original binded branch, the order of rebinding might get messed up
			node->trackOriginalBranch = true;
			child->trackOriginalBranch = true;
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

std::vector<ContourBinding*> SceneNode::contourPointsToRebind(std::vector<ContourBinding>& bindings) {
	std::vector<ContourBinding*> out;
	for (ContourBinding& b : bindings) {
		if (b.childNode->trackOriginalBranch && b.parentNode->trackOriginalBranch) {
			out.push_back(&b);
		}
	}
	return out;
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
void SceneNode::addNewBranch(SceneNode* node, ContourBinding* contour) {
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
		// need to accumulate all the parents' rotatation matrix until root and take the inverse
		newNode->localRotation = rotationMatrix * glm::toMat4(glm::inverse(accumulateRotationToRoot(node)));

		newNode->S = node->S;   // controls how fast or slow this new node will grow
		newNode->rotationAngle = 0;
		newNode->animationDirection = node->animationDirection;
		newNode->animationAngle = 0.f;
		newNode->animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		newNode->animationScaling = 1;
		newNode->animateScaling = glm::mat4(1.0);

		// Connect to tree
		newNode->parent = node;
		node->addChild(newNode);
		node->addBranch = false;
		// to add new branch
		newNode->addBranch = true;
	}
	else {
		for (SceneNode* child : node->children) {
			addNewBranch(child, contour);
		}
	}
}

int findContourIndex(const std::vector<ContourBinding>& bindings, const ContourBinding* target) {
	for (size_t i = 0; i < bindings.size(); ++i) {
		if (&bindings[i] == target) {
			return static_cast<int>(i); 
		}
	}
	return -1; 
}

void SceneNode::rebindContourToNewBranch(SceneNode* node, ContourBinding* contour, std::vector<ContourBinding>& bindings, std::vector<ContourBinding*>& toRebind) {
	// find the contour points to rebind
	std::vector<ContourBinding*> rebind;
	bool leftSide = false;
	bool rightSide = false;
	for (int i = 0; i < toRebind.size(); i++) {
		if (contour->contourPoint == toRebind[i]->contourPoint) {
			// assuming that toRebind will always have odd number of points because left + direct contour + right
			if (i < toRebind.size() / 2) {
				rebind.assign(toRebind.begin(), toRebind.begin() + (toRebind.size() / 2) + 1);  // before contour
				leftSide = true;
			}
			else {
				rebind.assign(toRebind.begin() + (toRebind.size() / 2), toRebind.end());   // after contour
				rightSide = true;
			}
		}
	}
	// divide the group of points to the left and right (not including the contour itself)
	std::vector<ContourBinding*> left;
	std::vector<ContourBinding*> right;
	int index = -1;
	int leftBranchingPointIndex = -1;
	int rightBranchingPointIndex = -1;
	for (int i = 0; i < rebind.size(); i++) {
		if (rebind[i]->contourPoint == contour->contourPoint) {  
			index = i;
		}
	}
	if (index != -1) {
		// left 
		left.assign(rebind.begin(),rebind.begin() + index);
		// right
		right.assign(rebind.begin() + index + 1, rebind.end());
	}
	for (int i = 0; i < rebind.size(); i++) {
		std::cout << glm::to_string(rebind[i]->contourPoint) << std::endl;
	}
	std::cout << index << std::endl;
	// rebind the contour points
	if (node->addBranch) {
		//for (int i = 1; i < rebind.size() - 1; i++) {   // want to not rebind the first and the last points of rebind
		//	rebind[i]->parentNode = node->parent;
		//	rebind[i]->childNode = node;
		//	rebind[i]->t = (float)i/rebind.size();
		//	rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
		//	rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
		//}

		leftBranchingPointIndex = (left.size()/2);
		rightBranchingPointIndex = index + (right.size()/2);
		// branching point rebinding
		if (leftBranchingPointIndex != -1) {
			rebind[leftBranchingPointIndex]->parentNode = node->parent->parent;
			rebind[leftBranchingPointIndex]->childNode = node->parent;
			rebind[leftBranchingPointIndex]->t = 1;
			rebind[leftBranchingPointIndex]->closestPoint = rebind[leftBranchingPointIndex]->t * rebind[leftBranchingPointIndex]->childNode->globalTransformation[3] + (1 - rebind[leftBranchingPointIndex]->t) * rebind[leftBranchingPointIndex]->parentNode->globalTransformation[3];
			rebind[leftBranchingPointIndex]->previousAnimateInverse = glm::inverse(rebind[leftBranchingPointIndex]->t * rebind[leftBranchingPointIndex]->childNode->globalTransformation + (1 - rebind[leftBranchingPointIndex]->t) * rebind[leftBranchingPointIndex]->parentNode->globalTransformation);
		}
		if (rightBranchingPointIndex != -1) {
			rebind[rightBranchingPointIndex]->parentNode = node->parent->parent;
			rebind[rightBranchingPointIndex]->childNode = node->parent;
			rebind[rightBranchingPointIndex]->t = 1;
			rebind[rightBranchingPointIndex]->closestPoint = rebind[rightBranchingPointIndex]->t * rebind[rightBranchingPointIndex]->childNode->globalTransformation[3] + (1 - rebind[rightBranchingPointIndex]->t) * rebind[rightBranchingPointIndex]->parentNode->globalTransformation[3];
			rebind[rightBranchingPointIndex]->previousAnimateInverse = glm::inverse(rebind[rightBranchingPointIndex]->t * rebind[rightBranchingPointIndex]->childNode->globalTransformation + (1 - rebind[rightBranchingPointIndex]->t) * rebind[rightBranchingPointIndex]->parentNode->globalTransformation);
		}
		// direct contour rebinding
		if (index != -1) {
			rebind[index]->parentNode = node->parent;
			rebind[index]->childNode = node;
			rebind[index]->t = 1.f;
			rebind[index]->closestPoint = rebind[index]->t * rebind[index]->childNode->globalTransformation[3] + (1 - rebind[index]->t) * rebind[index]->parentNode->globalTransformation[3];
			rebind[index]->previousAnimateInverse = glm::inverse(rebind[index]->t * rebind[index]->childNode->globalTransformation + (1 - rebind[index]->t) * rebind[index]->parentNode->globalTransformation);
		}
		// interpolate points in between
		if (leftSide) {  // new branch added on the left
			int rightMostIndex = rebind.size() - 1;
			for (int i = 1; i < leftBranchingPointIndex; i++) {   
				rebind[i]->parentNode = node->parent->parent;
				rebind[i]->childNode = node->parent;
				rebind[i]->t = 1 - ((float)i / leftBranchingPointIndex);
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {    
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = (float)i / index;
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = index + 1; i < rightBranchingPointIndex; i++) {
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = (float)i / rightBranchingPointIndex;
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = rightBranchingPointIndex + 1; i < rebind.size() - 1; i++) {
				rebind[i]->parentNode = rebind[rightMostIndex]->parentNode;
				rebind[i]->childNode = rebind[rightMostIndex]->childNode;
				rebind[i]->t = (float)i / rebind.size();
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
		}
		if (rightSide) {    // new branch added on the right
			int leftMostIndex = 0;
			for (int i = 1; i < leftBranchingPointIndex; i++) {   
				rebind[i]->parentNode = rebind[leftMostIndex]->parentNode;
				rebind[i]->childNode = rebind[leftMostIndex]->childNode;
				rebind[i]->t = 1 - ((float)i / leftBranchingPointIndex);
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {    
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = (float)i / (index);
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = index + 1; i < rightBranchingPointIndex; i++) {
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = 1 - ((float)i / (rightBranchingPointIndex));
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = rightBranchingPointIndex + 1; i < rebind.size(); i++) {
				rebind[i]->parentNode = node->parent->parent;
				rebind[i]->childNode = node->parent;
				rebind[i]->t = 1 - ((float)i / (rebind.size()));
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
		}
		// reset new branch
		node->addBranch = false;
		for (int i = 0; i < rebind.size(); i++) {
			std::cout << glm::to_string(rebind[i]->contourPoint) << std::endl;
			std::cout << glm::to_string(rebind[i]->parentNode->globalTransformation[3]) << std::endl;
			std::cout << glm::to_string(rebind[i]->childNode->globalTransformation[3]) << std::endl;
			std::cout << rebind[i]->t << std::endl;
		}
		std::cout << "---------------------" << std::endl;
	}
	else {
		for (SceneNode* child : node->children) {
			rebindContourToNewBranch(child, contour, bindings, toRebind);
		}
	}
}

//void SceneNode::rebindContourToNewBranch(SceneNode* node, ContourBinding* contour, std::vector<ContourBinding>& bindings, std::vector<ContourBinding*>& toRebind) {
//	// find the contour points to rebind
//	std::vector<ContourBinding*> rebind;
//	for (int i = 0; i < toRebind.size(); i++) {
//		if (contour->contourPoint == toRebind[i]->contourPoint) {
//			if (i < toRebind.size() / 2) rebind.assign(toRebind.begin(), toRebind.begin() + (toRebind.size() / 2));  // before contour
//			else rebind.assign(toRebind.begin() + (toRebind.size() / 2) + 1, toRebind.end());   // after contour
//		}
//	}
//
//	// rebind the contour points
//	if (node->branchAdded) {
//		for (int i = 0; i < rebind.size(); i++) {
//			rebind[i]->parentNode = node->parent;
//			rebind[i]->childNode = node;
//			rebind[i]->t = (float)i / rebind.size();
//			rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
//			rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
//		}
//		contour->parentNode = node->parent;
//		contour->childNode = node;
//		contour->t = 1.f;
//		contour->closestPoint = contour->t * contour->childNode->globalTransformation[3] + (1 - contour->t) * contour->parentNode->globalTransformation[3];
//		contour->previousAnimateInverse = glm::inverse(contour->t * contour->childNode->globalTransformation + (1 - contour->t) * contour->parentNode->globalTransformation);
//		// reset new branch
//		node->branchAdded = false;
//	}
//	else {
//		for (SceneNode* child : node->children) {
//			rebindContourToNewBranch(child, contour, bindings, toRebind);
//		}
//	}
//}

//void SceneNode::rebindContourToNewBranch(SceneNode* node, ContourBinding* contour, std::vector<ContourBinding>& bindings, std::vector<ContourBinding>& toRebind) {
//	// rebind the contour point
//	if (node->branchAdded) {
//		// rebind point on the left
//		int index = findContourIndex(bindings, contour);
//		bindings[index - 1].parentNode = node->parent;
//		bindings[index - 1].childNode = node;
//		bindings[index - 1].t = 0.5f;
//		bindings[index - 1].closestPoint = bindings[index - 1].t * bindings[index - 1].childNode->globalTransformation[3] + (1 - bindings[index - 1].t) * bindings[index - 1].parentNode->globalTransformation[3];
//		bindings[index - 1].previousAnimateInverse = glm::inverse(bindings[index - 1].t * bindings[index - 1].childNode->globalTransformation + (1 - bindings[index - 1].t) * bindings[index - 1].parentNode->globalTransformation);
//
//		// rebind point on the right
//		bindings[index + 1].parentNode = node->parent;
//		bindings[index + 1].childNode = node;
//		bindings[index + 1].t = 0.5f;
//		bindings[index + 1].closestPoint = bindings[index + 1].t * bindings[index + 1].childNode->globalTransformation[3] + (1 - bindings[index + 1].t) * bindings[index + 1].parentNode->globalTransformation[3];
//		bindings[index + 1].previousAnimateInverse = glm::inverse(bindings[index + 1].t * bindings[index + 1].childNode->globalTransformation + (1 - bindings[index + 1].t) * bindings[index + 1].parentNode->globalTransformation);
//
//		// rebind point itself
//		contour->parentNode = node->parent;
//		contour->childNode = node;
//		contour->t = 1.f;
//		contour->closestPoint = contour->t * contour->childNode->globalTransformation[3] + (1 - contour->t) * contour->parentNode->globalTransformation[3];
//		contour->previousAnimateInverse = glm::inverse(contour->t * contour->childNode->globalTransformation + (1 - contour->t) * contour->parentNode->globalTransformation);
//
//		// reset new branch
//		node->branchAdded = false;
//	}
//	else {
//		for (SceneNode* child : node->children) {
//			rebindContourToNewBranch(child, contour, bindings, toRebind);
//		}
//	}
//}

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

	glm::vec3 leftOffset = rootPos - glm::vec3(0.415f, 0.25f, 0.0f);
	glm::vec3 rightOffset = rootPos + glm::vec3(0.415f, -0.25f, 0.0f);

	controlPoints.push_back(leftOffset);

	// leaf
	std::vector<SceneNode*> leaves;
	getLeafNodes(root, leaves);

	for (SceneNode* leaf : leaves) {
		glm::vec3 leafPos = leaf->globalTransformation[3];
		glm::vec3 leafParentPos = leaf->parent->globalTransformation[3];
		glm::vec3 dir = glm::normalize(leafPos - leafParentPos);
		glm::vec3 offsetPos = leafPos + dir * 0.15f;
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

	std::vector<glm::vec3> paddedPoints;
	glm::vec3 first = controlPoints[0] + (controlPoints[0] - controlPoints[1]);
	glm::vec3 last = controlPoints.back() + (controlPoints.back() - controlPoints[controlPoints.size() - 2]);

	paddedPoints.push_back(first);
	paddedPoints.insert(paddedPoints.end(), controlPoints.begin(), controlPoints.end());
	paddedPoints.push_back(last);

	for (size_t i = 0; i < paddedPoints.size() - 3; i++) {
		std::vector<glm::vec3> segmentPoints;

		for (int j = 0; j < pointsPerSegment; j++) {
			float t = float(j) / (pointsPerSegment);
			glm::vec3 pt = glm::catmullRom(
				paddedPoints[i],
				paddedPoints[i + 1],
				paddedPoints[i + 2],
				paddedPoints[i + 3],
				t
			);
			segmentPoints.push_back(pt);

			if (j == pointsPerSegment / 2) {
				float minDist = FLT_MAX;
				for (auto& [parent, child, index] : branches) {
					glm::vec3 P = parent->globalTransformation[3];
					glm::vec3 Q = child->globalTransformation[3];

					glm::vec3 closest = SceneNode::intersectionPoint(P, Q, pt);
					float dist = glm::length(closest - pt);

					if (dist < minDist) {
						minDist = dist;
						segment = { parent, child };
					}
				}
			}
		}
		groupedContourPoints.push_back({ segmentPoints, segment });

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

	for (size_t i = 0; i < controlPoints.size() - 1; i++) {
		std::vector<glm::vec3> segmentPoints;

		for (int j = 0; j < pointsPerSegment; j++) {
			float t = float(j) / pointsPerSegment;
			glm::vec3 pt = (1.0f - t) * controlPoints[i] + t * controlPoints[i + 1];
			segmentPoints.push_back(pt);

			if (j == pointsPerSegment / 2) {
				float minDist = FLT_MAX;
				for (auto& [parent, child, index] : branches) {
					glm::vec3 P = parent->globalTransformation[3];
					glm::vec3 Q = child->globalTransformation[3];

					glm::vec3 closest = SceneNode::intersectionPoint(P, Q, pt);
					float dist = glm::length(closest - pt);

					if (dist < minDist) {
						minDist = dist;
						segment = { parent, child };
					}
				}
			}
		}
		groupedContourPoints.push_back({ segmentPoints, segment });

		// last contour point
		if (i == controlPoints.size() - 2) groupedContourPoints.push_back({ {controlPoints[i + 1]}, segment });
	}

	return groupedContourPoints;
}

std::vector<ContourBinding> SceneNode::bindInterpolatedContourToBranches(std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>>& contourPoints) {
	std::vector<ContourBinding> bindings;
	float t = 0;
	// segment
	for (int i = 0; i < contourPoints.size(); i++) {
		// individual points in the segment
		for (int j = 0; j < contourPoints[i].first.size(); j++) {
			SceneNode* parent = contourPoints[i].second.first;
			SceneNode* child = contourPoints[i].second.second;
			glm::vec3 P = parent->globalTransformation[3];
			glm::vec3 Q = child->globalTransformation[3];
			ContourBinding bestBinding;
			if (i % 2 == 1) t = 1 - (j / ((float)contourPoints[i].first.size()));
			else t = (j / ((float)contourPoints[i].first.size()));
			// last group has one contour point (last contour point)
			if (contourPoints[i].first.size() == 1) t = 0;

			// use non-linear (easing) function for t -> must be monotonic
			//t = (3 * t * t) - (2 * t * t * t);
			//t = 1 - cos((t * 3.141592653589793) / 2);
			//t = sin((t * 3.141592653589793) / 2);
			//t = 1 - exp(-5 * t);
			//t = t < 0.5
			//	? std::pow(2 * t, 2) / 2
			//	: 1 - std::pow(2 - 2 * t, 2) / 2;
			//t = 1.0 / (1.0 + exp(-5 * (t - 0.5)));;    // k value controls indentation, logistic function
			//t = t * t;

			bestBinding = { parent, child, contourPoints[i].first[j], t, t * Q + (1 - t) * P, glm::inverse(t * child->globalTransformation + (1 - t) * parent->globalTransformation)};
			bindings.push_back(bestBinding);
		}
	}

	return bindings;
}

// find the children of common ancestor
std::tuple<SceneNode*, SceneNode*> SceneNode::findChildrenOfFirstCommonAncestorFromRoot(
	SceneNode* root,
	const ContourBinding& a,
	const ContourBinding& b) {
	if (a.childNode == b.childNode) {
		return { a.childNode, b.childNode } ;
	}

	auto buildPathToRoot = [](SceneNode* node) {
		std::vector<SceneNode*> path;
		while (node) {
			path.push_back(node);
			node = node->parent;
		}
		return path;
		};

	std::vector<SceneNode*> pathA = buildPathToRoot(a.childNode);
	std::vector<SceneNode*> pathB = buildPathToRoot(b.childNode);
	std::reverse(pathA.begin(), pathA.end());
	std::reverse(pathB.begin(), pathB.end());

	size_t minSize = std::min(pathA.size(), pathB.size());
	size_t index = 0;

	for (size_t i = 0; i < minSize; i++) {
		if (pathA[i]->globalTransformation[3] == pathB[i]->globalTransformation[3]) {
			index += 1;
		}
		else {
			break;
		}
	}
	return { pathA[index], pathB[index] };
}

// finding the closest point from R (on contour) to the branch segment PQ 
glm::vec3 SceneNode::intersectionPoint(glm::vec3 P, glm::vec3 Q, glm::vec3 R) {
	float t = dot(Q - P, R - P) / dot(Q - P, Q - P);
	if (t <= 0) return P;
	else if (t >= 1) return Q;
	else {
		return P + t * (Q - P);
	}
}

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

// interpolate branches 
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

// add contour points to newly inserted branch nodes
std::vector<ContourBinding> SceneNode::addNewContourToBindToNewBranchNode(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs) {
	std::vector<ContourBinding> newBindingSet;
	for (int i = 0; i < bindings.size() - 1; i++) {
		// two consecutive contour points belong to different branches (so a newly inserted branch node exists)
		// assuming that there is only one newly added branch node between the existing ones

		// left side
		if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].childNode == bindings[i + 1].parentNode && bindings[i].t != 1 && bindings[i].t != 0) {
			glm::vec3 neighborParent = bindings[i].parentNode->globalTransformation[3];
			glm::vec3 neighborChild = bindings[i].childNode->globalTransformation[3];
			float t = 1;
			glm::mat4 previousiInverseAnimationMat = glm::inverse(t * bindings[i].childNode->globalTransformation + (1 - t) * bindings[i].parentNode->globalTransformation);
			glm::vec3 closestPoint = t * bindings[i].childNode->globalTransformation[3] + (1 - t) * bindings[i].parentNode->globalTransformation[3];
			// find the t for the branch node
			float branchT = glm::length(bindings[i].parentNode->globalTransformation[3] - bindings[i].childNode->globalTransformation[3]) / glm::length(bindings[i].parentNode->globalTransformation[3] - bindings[i + 1].childNode->globalTransformation[3]);
			glm::vec3 newPoint = branchT * bindings[i].contourPoint + (1 - branchT) * bindings[i + 1].contourPoint;
			newBindingSet.push_back(bindings[i]);
			newBindingSet.push_back({ bindings[i].parentNode, bindings[i].childNode, newPoint, t, closestPoint, previousiInverseAnimationMat });
		}
		// right side
		else if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].parentNode == bindings[i + 1].childNode && bindings[i + 1].t != 1 && bindings[i + 1].t != 0) {
			glm::vec3 neighborParent = bindings[i + 1].parentNode->globalTransformation[3];
			glm::vec3 neighborChild = bindings[i + 1].childNode->globalTransformation[3];
			float t = 1;
			glm::mat4 previousiInverseAnimationMat = glm::inverse(t * bindings[i + 1].childNode->globalTransformation + (1 - t) * bindings[i + 1].parentNode->globalTransformation);
			glm::vec3 closestPoint = t * bindings[i + 1].childNode->globalTransformation[3] + (1 - t) * bindings[i + 1].parentNode->globalTransformation[3];
			// find the t for the branch node
			float branchT = glm::length(bindings[i + 1].parentNode->globalTransformation[3] - bindings[i + 1].childNode->globalTransformation[3]) / glm::length(bindings[i + 1].parentNode->globalTransformation[3] - bindings[i].childNode->globalTransformation[3]);
			glm::vec3 newPoint = branchT * bindings[i + 1].contourPoint + (1 - branchT) * bindings[i].contourPoint;
			newBindingSet.push_back(bindings[i]);
			newBindingSet.push_back({ bindings[i + 1].parentNode, bindings[i + 1].childNode, newPoint, t, closestPoint, previousiInverseAnimationMat });
		}
		else {
			newBindingSet.push_back(bindings[i]);
		}
	}
	newBindingSet.push_back(bindings[bindings.size() - 1]);

	return newBindingSet;
}

// add a new point to the curve
std::vector<ContourBinding> SceneNode::addContourPoints(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs) {
	std::vector<ContourBinding> newBindingSet;
	// arbitrary threshold
	float threshold = 0.15f;
	for (int i = 0; i < bindings.size() - 1; i++) {
		float distance = glm::length(bindings[i + 1].contourPoint - bindings[i].contourPoint);
		if (distance >= threshold) {
			glm::vec3 newPoint = glm::mix(bindings[i].contourPoint, bindings[i + 1].contourPoint, 0.5f);
			// special case, if they are both 1 (different branch), then connect to the child branch
			if ((bindings[i].t == 1 && bindings[i + 1].t == 1) && (bindings[i].childNode != bindings[i + 1].childNode)) {
				ContourBinding& neighbor = (bindings[i].childNode == bindings[i + 1].parentNode) ? bindings[i + 1] : bindings[i];
				glm::vec3 neighborParent = neighbor.parentNode->globalTransformation[3];
				glm::vec3 neighborChild = neighbor.childNode->globalTransformation[3];
				float t = neighbor.t / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(t * neighbor.childNode->globalTransformation + (1 - t) * neighbor.parentNode->globalTransformation);
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ neighbor.parentNode, neighbor.childNode, newPoint, t, t * neighbor.childNode->globalTransformation[3] + (1 - t) * neighbor.parentNode->globalTransformation[3], previousiInverseAnimationMat });
			}
			// special case, belong to different branches; connect to the parent branch -> need to explicitly check that they belong to different branches
			else if ((bindings[i].t == 1 || bindings[i + 1].t == 1) && (bindings[i].childNode != bindings[i + 1].childNode)) {
				ContourBinding& neighbor = (bindings[i].t != 1) ? bindings[i] : bindings[i + 1];
				glm::vec3 neighborParent = neighbor.parentNode->globalTransformation[3];
				glm::vec3 neighborChild = neighbor.childNode->globalTransformation[3];
				float t = neighbor.t / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(t * neighbor.childNode->globalTransformation + (1 - t) * neighbor.parentNode->globalTransformation);
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ neighbor.parentNode, neighbor.childNode, newPoint, t, t * neighbor.childNode->globalTransformation[3] + (1 - t) * neighbor.parentNode->globalTransformation[3], previousiInverseAnimationMat});
			}
			else {
				glm::vec3 neighborParent = bindings[i].parentNode->globalTransformation[3];
				glm::vec3 neighborChild = bindings[i].childNode->globalTransformation[3];
				glm::vec3 proj = intersectionPoint(neighborParent, neighborChild, newPoint);
				float dist1 = glm::length(newPoint - proj);
				float t = (bindings[i].t + bindings[i + 1].t) / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(t * bindings[i].childNode->globalTransformation + (1 - t) * bindings[i].parentNode->globalTransformation);
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ bindings[i].parentNode, bindings[i].childNode, newPoint, t, glm::mix(bindings[i].closestPoint, bindings[i + 1].closestPoint, 0.5f), previousiInverseAnimationMat });
			}
		}
		else {
			newBindingSet.push_back(bindings[i]);
		}
	}
	newBindingSet.push_back(bindings[bindings.size() - 1]);

	return newBindingSet;
}

//// relative transformation between frame (global frame)
//void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings) {
//	// need the previous frame's animation matrix and current frame's animation matrix
//	// update the contour point every frame (in the ContourBinding) so that you just apply the matrix to the contour point
//	// this is in the "global" frame
//	for (auto& binding : bindings) {
//		glm::mat4 animatedPosMat = binding.t * binding.childNode->globalTransformation + (1 - binding.t) * (binding.parentNode->globalTransformation);
//		printMat4(animatedPosMat);
//		binding.contourPoint = animatedPosMat * binding.previousAnimateInverse * glm::vec4(binding.contourPoint, 1.0f);
//		binding.previousAnimateInverse = glm::inverse(binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation);
//		binding.closestPoint = binding.t * binding.childNode->globalTransformation[3] + (1 - binding.t) * (binding.parentNode->globalTransformation[3]);
//	}
//}

void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings) {
	//// Open the file (append mode so you don’t overwrite previous frames)
	//std::ofstream file("animated_matrices_no_division_scaling2.txt", std::ios::trunc);
	//if (!file) {
	//	std::cerr << "Could not open file for writing.\n";
	//	return;
	//}
	//int index = 0;
	for (auto& binding : bindings) {
	//	glm::mat4 matrix = binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation;
	//	// Write matrix to file
	//	if (index == 9 && animated) {
	//		file << binding.contourPoint.x << " " << binding.contourPoint.y << " " << binding.contourPoint.z << "\n";
	//		for (int i = 0; i < 4; ++i) {
	//			file << "[ ";
	//			for (int j = 0; j < 4; ++j) {
	//				file << matrix[i][j] << " ";
	//			}
	//			file << "]\n";
	//		}
	//		file << "\n";
	//		matrix *= binding.previousAnimateInverse;
	//		for (int i = 0; i < 4; ++i) {
	//			file << "[ ";
	//			for (int j = 0; j < 4; ++j) {
	//				file << matrix[i][j] << " ";
	//			}
	//			file << "]\n";
	//		}
	//		file << "\n";
	//		animated = false;
	//	}

		// Update binding
		glm::mat4 animatedPosMat = binding.t * binding.childNode->globalTransformation +
			(1 - binding.t) * binding.parentNode->globalTransformation;
		binding.contourPoint = animatedPosMat * binding.previousAnimateInverse * glm::vec4(binding.contourPoint, 1.0f);
		binding.previousAnimateInverse = glm::inverse(binding.t * binding.childNode->globalTransformation +
			(1 - binding.t) * binding.parentNode->globalTransformation);
		binding.closestPoint = binding.t * binding.childNode->globalTransformation[3] +
			(1 - binding.t) * (binding.parentNode->globalTransformation[3]);
		//index += 1;
	}

	//file << "-------------------------\n";
	//file.close(); // Close the file after writing
}


void SceneNode::handleMouseClick(double xpos, double ypos, int screenWidth, int screenHeight, glm::mat4 view, glm::mat4 projection, std::vector<glm::vec3> contourPoints, CPU_Geometry geom) {
	glm::vec2 clickPos;
	clickPos = glm::vec2(xpos, ypos);
	clickPos += glm::vec2(0.5f, 0.5f);
	clickPos /= (glm::vec2(screenWidth, screenHeight));
	clickPos = glm::vec2(clickPos.x, 1.0f - clickPos.y);
	clickPos *= 2.0f;
	clickPos -= glm::vec2(1.0f, 1.0f);

	contourPoints.push_back(glm::vec3(clickPos, 0.0f));
	std::cout << glm::to_string(clickPos) << std::endl;

}
