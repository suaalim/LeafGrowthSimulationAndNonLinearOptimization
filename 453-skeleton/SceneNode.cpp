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
//void printMat4(const glm::mat4& mat) {
//	for (int i = 0; i < 4; i++) {
//		std::cout << "| ";
//		for (int j = 0; j < 4; j++) {
//			std::cout << mat[j][i] << "\t";
//		}
//		std::cout << "|\n";
//	}
//	std::cout << std::endl;
//}

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

	copy->parent = parent;

	// clone and original mapping
	nodeMap[node] = copy;

	for (SceneNode* child : node->children) {
		SceneNode* childCopy = cloneSceneNode(child, copy, nodeMap);
		copy->addChild(childCopy);
	}

	return copy;
}

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
			binding.newBranchBinding
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

void SceneNode::rebindContourWithBrokenBranch(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments, int& i, std::vector<ContourBinding>& bindings) {
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
				if (totalDistance < minTotalDistance && projected != glm::vec3(0.f, 0.f, 0.f)) {   // don't want to add a new branch from the root
					minTotalDistance = totalDistance;
					// special case for tip of the branch, to preserve the tip of the main axis
					if (t == 1) {
						t = 0.85f;
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

//void resetBool(SceneNode* root) {
//	root->trackOriginalBranch = false;
//	for (SceneNode* child : root->children) resetBool(child);
//}

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
SceneNode* SceneNode::addNewBranch(SceneNode* node, ContourBinding* contour) {
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

		newNode->S = node->S * 2;   // controls how fast or slow this new node will grow
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

		return newNode;
	}
	else {
		for (SceneNode* child : node->children) {
			SceneNode* result = addNewBranch(child, contour);
			if (result) return result;  
		}
	}
	return nullptr; 
}

std::vector<size_t> SceneNode::contourBindingIndicesToRebind(const std::vector<ContourBinding>& bindings, SceneNode* root) {
	std::vector<size_t> indices;
	for (size_t i = 1; i < bindings.size() - 1; i++) {
		if (bindings[i].childNode->trackOriginalBranch && bindings[i].parentNode->trackOriginalBranch) {
			indices.push_back(i);
			if (bindings[i].newBranchBinding) return {};  // do not want to add new branch; return empty vector
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
				float side = parentBranch.x * vec.y - parentBranch.y * vec.x;
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
	// branching point binding is determined by distance (ratio)
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
			bindings[i].t = glm::clamp(((float)(i) / leftBranchingPointIndex), 0.f, 1.f);
			if (bindings[i].t < 0.05 && newNode->parent->parent->parent != NULL) {  // snap if t is close to 0
				bindings[i].parentNode = newNode->parent->parent->parent;
				bindings[i].childNode = newNode->parent->parent;
				bindings[i].t = 1;
			}
			bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
			bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
		}
		if (leftBranchingPointIndex != -1) {
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {
				bindings[i].parentNode = newNode->parent;
				bindings[i].childNode = newNode;
				bindings[i].t = glm::clamp((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex), 0.f, 1.f);
				if (bindings[i].t < 0.05 && newNode->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent;
					bindings[i].childNode = newNode->parent;
					bindings[i].t = 1;
				}
				bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
				bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
				bindings[i].newBranchBinding = true;
			}
		}
		for (int i = index + 1; i < rightBranchingPointIndex; i++) {
			bindings[i].parentNode = newNode->parent;
			bindings[i].childNode = newNode;
			bindings[i].t = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index)), 0.f, 1.f);
			if (bindings[i].t < 0.05 && newNode->parent->parent != NULL) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].t = 1;
			}
			bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
			bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
			bindings[i].newBranchBinding = true;
		}
		if (rightBranchingPointIndex != -1) {
			for (int i = rightBranchingPointIndex + 1; i <= rebind.back(); i++) {
				bindings[i].parentNode = bindings[rightMostIndex].childNode->parent;
				bindings[i].childNode = bindings[rightMostIndex].childNode;
				bindings[i].t = glm::clamp((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex), 0.f, 1.f);
				if (bindings[i].t < 0.05 && bindings[rightMostIndex].parentNode->parent != NULL) {
					bindings[i].parentNode = bindings[rightMostIndex].parentNode->parent;
					bindings[i].childNode = bindings[rightMostIndex].parentNode;
					bindings[i].t = 1;
				}
				bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
				bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
			}
		}
	}
	if (rightSide) {    // new branch added on the right
		int leftMostIndex = rebind.front();
		for (int i = rebind.front(); i < leftBranchingPointIndex; i++) {
			bindings[i].parentNode = bindings[leftMostIndex].parentNode;
			bindings[i].childNode = bindings[leftMostIndex].childNode;
			bindings[i].t = glm::clamp(1 - ((float)(i - rebind.front()) / (leftBranchingPointIndex - rebind.front())), 0.f, 1.f);
			if (bindings[i].t < 0.05 && bindings[leftMostIndex].parentNode->parent != NULL) {
				bindings[i].parentNode = bindings[leftMostIndex].parentNode->parent;
				bindings[i].childNode = bindings[leftMostIndex].parentNode;
				bindings[i].t = 1;
			}
			bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
			bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
		}
		if (leftBranchingPointIndex != -1) {
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {
				bindings[i].parentNode = newNode->parent;
				bindings[i].childNode = newNode;
				bindings[i].t = glm::clamp(((float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex)), 0.f, 1.f);
				if (bindings[i].t < 0.05 && newNode->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent;
					bindings[i].childNode = newNode->parent;
					bindings[i].t = 1;
				}
				bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
				bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
				bindings[i].newBranchBinding = true;
			}
		}
		for (int i = index + 1; i < rightBranchingPointIndex; i++) {
			bindings[i].parentNode = newNode->parent;
			bindings[i].childNode = newNode;
			bindings[i].t = glm::clamp(1 - ((float)(i - index) / (rightBranchingPointIndex - index)), 0.f, 1.f);
			if (bindings[i].t < 0.05 && newNode->parent->parent != NULL) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].t = 1;
			}
			bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
			bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
			bindings[i].newBranchBinding = true;
		}
		if (rightBranchingPointIndex != -1) {
			for (int i = rightBranchingPointIndex + 1; i <= rebind.back(); i++) {
				bindings[i].parentNode = newNode->parent->parent;
				bindings[i].childNode = newNode->parent;
				bindings[i].t = glm::clamp(1 - ((float)(i - rightBranchingPointIndex) / (rebind.back() - rightBranchingPointIndex)), 0.f, 1.f);
				if (bindings[i].t < 0.05 && newNode->parent->parent->parent != NULL) {
					bindings[i].parentNode = newNode->parent->parent->parent;
					bindings[i].childNode = newNode->parent->parent;
					bindings[i].t = 1;
				}
				bindings[i].closestPoint = bindings[i].t * bindings[i].childNode->globalTransformation[3] + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation[3];
				bindings[i].previousAnimateInverse = glm::inverse(bindings[i].t * bindings[i].childNode->globalTransformation + (1 - bindings[i].t) * bindings[i].parentNode->globalTransformation);
			}
		}
	}
	// branching point rebinding
	if (leftBranchingPointIndex != -1) {
		bindings[leftBranchingPointIndex].parentNode = newNode->parent->parent;
		bindings[leftBranchingPointIndex].childNode = newNode->parent;
		bindings[leftBranchingPointIndex].t = 1;
		bindings[leftBranchingPointIndex].closestPoint = bindings[leftBranchingPointIndex].t * bindings[leftBranchingPointIndex].childNode->globalTransformation[3] + (1 - bindings[leftBranchingPointIndex].t) * bindings[leftBranchingPointIndex].parentNode->globalTransformation[3];
		bindings[leftBranchingPointIndex].previousAnimateInverse = glm::inverse(bindings[leftBranchingPointIndex].t * bindings[leftBranchingPointIndex].childNode->globalTransformation + (1 - bindings[leftBranchingPointIndex].t) * bindings[leftBranchingPointIndex].parentNode->globalTransformation);
		bindings[leftBranchingPointIndex].newBranchBinding = true;
	}
	if (rightBranchingPointIndex != -1) {
		bindings[rightBranchingPointIndex].parentNode = newNode->parent->parent;
		bindings[rightBranchingPointIndex].childNode = newNode->parent;
		bindings[rightBranchingPointIndex].t = 1;
		bindings[rightBranchingPointIndex].closestPoint = bindings[rightBranchingPointIndex].t * bindings[rightBranchingPointIndex].childNode->globalTransformation[3] + (1 - bindings[rightBranchingPointIndex].t) * bindings[rightBranchingPointIndex].parentNode->globalTransformation[3];
		bindings[rightBranchingPointIndex].previousAnimateInverse = glm::inverse(bindings[rightBranchingPointIndex].t * bindings[rightBranchingPointIndex].childNode->globalTransformation + (1 - bindings[rightBranchingPointIndex].t) * bindings[rightBranchingPointIndex].parentNode->globalTransformation);
		bindings[rightBranchingPointIndex].newBranchBinding = true;
	}
	// direct contour rebinding
	if (index != -1) {
		bindings[index].parentNode = newNode->parent;
		bindings[index].childNode = newNode;
		bindings[index].t = 1;
		bindings[index].closestPoint = bindings[index].t * bindings[index].childNode->globalTransformation[3] + (1 - bindings[index].t) * bindings[index].parentNode->globalTransformation[3];
		bindings[index].previousAnimateInverse = glm::inverse(bindings[index].t * bindings[index].childNode->globalTransformation + (1 - bindings[index].t) * bindings[index].parentNode->globalTransformation);
		bindings[index].newBranchBinding = true;
	}

	//// if leftBranchingPointIndex == index, add a new point in between
	//if (leftBranchingPointIndex == index) {
	//	glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
	//	glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
	//	float t = 1;
	//	glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
	//	glm::vec3 closestPoint = t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
	//	// find the t for the branch newNode
	//	float branchT = glm::length(bindings[leftBranchingPointIndex].parentNode->globalTransformation[3] - bindings[leftBranchingPointIndex].childNode->globalTransformation[3]) / glm::length(bindings[leftBranchingPointIndex].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
	//	glm::vec3 newPoint = branchT * bindings[leftBranchingPointIndex].contourPoint + (1 - branchT) * bindings[index].contourPoint;
	//	bindings.insert(bindings.begin() + index, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat });
	//}
	//// if rightBranchingPointIndex == index, add a new point in between
	//if (rightBranchingPointIndex == index) {
	//	glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
	//	glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
	//	float t = 1;
	//	glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
	//	glm::vec3 closestPoint = t * t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
	//	// find the t for the branch newNode
	//	float branchT = glm::length(bindings[rightBranchingPointIndex].parentNode->globalTransformation[3] - bindings[rightBranchingPointIndex].childNode->globalTransformation[3]) / glm::length(bindings[rightBranchingPointIndex].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
	//	glm::vec3 newPoint = branchT * bindings[rightBranchingPointIndex].contourPoint + (1 - branchT) * bindings[index].contourPoint;
	//	bindings.insert(bindings.begin() + rightBranchingPointIndex, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat });
	//}

	// if leftBranchingPointIndex or rightBranchingPointIndex is -1, insert a new point
	if (leftBranchingPointIndex == -1 && leftSide) {
		glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
		glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
		glm::vec3 closestPoint = t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index - 1].parentNode->globalTransformation[3] - bindings[index - 1].childNode->globalTransformation[3]) / glm::length(bindings[index - 1].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index - 1].contourPoint + (1 - branchT) * bindings[index].contourPoint;
		bindings.insert(bindings.begin() + index, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true });
	}
	if (leftBranchingPointIndex == -1 && rightSide) {
		glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
		glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
		glm::vec3 closestPoint = t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]) / glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index - 1].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index].contourPoint + (1 - branchT) * bindings[index - 1].contourPoint;
		bindings.insert(bindings.begin() + index, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true });
	}
	if (rightBranchingPointIndex == -1 && leftSide) {
		glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
		glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
		glm::vec3 closestPoint = t * t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]) / glm::length(bindings[index].parentNode->globalTransformation[3] - bindings[index + 1].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index].contourPoint + (1 - branchT) * bindings[index + 1].contourPoint;
		bindings.insert(bindings.begin() + index + 1, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true });
	}
	if (rightBranchingPointIndex == -1 && rightSide) {
		glm::vec3 neighborParent = newNode->parent->parent->globalTransformation[3];
		glm::vec3 neighborChild = newNode->parent->globalTransformation[3];
		float t = 1;
		glm::mat4 previousiInverseAnimationMat = glm::inverse(t * newNode->parent->globalTransformation + (1 - t) * newNode->parent->parent->globalTransformation);
		glm::vec3 closestPoint = t * t * newNode->parent->globalTransformation[3] + (1 - t) * newNode->parent->parent->globalTransformation[3];
		// find the t for the branch newNode
		float branchT = glm::length(bindings[index + 1].parentNode->globalTransformation[3] - bindings[index + 1].childNode->globalTransformation[3]) / glm::length(bindings[index + 1].parentNode->globalTransformation[3] - bindings[index].childNode->globalTransformation[3]);
		glm::vec3 newPoint = branchT * bindings[index + 1].contourPoint + (1 - branchT) * bindings[index].contourPoint;
		bindings.insert(bindings.begin() + index + 1, { newNode->parent->parent, newNode->parent, newPoint, t, closestPoint, previousiInverseAnimationMat, true });
	}

	// reset new branch
	newNode->addBranch = false;
}
/*
// non-index based
std::vector<ContourBinding*> SceneNode::contourPointsToRebind(std::vector<ContourBinding>& bindings, SceneNode* root) {
	std::vector<ContourBinding*> out;
	for (ContourBinding& b : bindings) {
		if (b.childNode->trackOriginalBranch && b.parentNode->trackOriginalBranch) {
			out.push_back(&b);
		}
	}
	resetBool(root);
	return out;
}

// returns the index of the contour point that will be binded to the branching point
std::pair<int, int> branchingPointContour(float branchLength, float leftContourLength, float rightContourLength, int leftSize, int rightSize) {
	// take the smaller ratio out of the two (left and right)
	float leftDistance = leftSize / 2;
	float rightDistance = rightSize / 2;
	// guarantees equal number of contour points to be binded to the new branch (as long as left and right have at least 1 element)
	std::pair<int, int> indices = (leftSize <= rightSize) ? std::make_pair<int, int>((int)leftDistance, (leftSize - 1 - (int)leftDistance)) : std::make_pair<int, int>((leftSize - 1 - (int)rightDistance), (int)rightDistance);
	//if (leftDistance <= rightDistance && leftSide) return indices = std::make_pair<int, int>(leftDistance, leftDistance);
	//else if (leftDistance <= rightDistance && rightSide) return indices = std::make_pair<int, int>(leftDistance, leftDistance);
	//else if (leftDistance >= rightDistance && leftSide) return indices = std::make_pair<int, int>((leftSize - 1 - (int)rightDistance), rightDistance);
	//else if (leftDistance >= rightDistance && rightSide) return indices = std::make_pair<int, int>((leftSize - 1 - (int)rightDistance), rightDistance);
	return indices;
}

void SceneNode::rebindContourToNewBranch(SceneNode* node, ContourBinding* contour, int division, std::vector<ContourBinding*>& toRebind) {
	// find the contour points to rebind
	std::vector<ContourBinding*> rebind;
	std::vector<ContourBinding*> leftRebind;
	std::vector<ContourBinding*> rightRebind;
	bool leftSide = false;
	bool rightSide = false;

	for (int i = 0; i < toRebind.size(); i++) {  // dividing in the middle does not guarantee left/right half of the contour; need to compare x values
		if (contour->contourPoint == toRebind[i]->contourPoint) {
			// Split based on x-coordinate of the contour point
			for (ContourBinding* b : toRebind) {
				if (b->contourPoint.x < 0) {
					leftRebind.push_back(b);
				}
				else if (b->contourPoint.x > 0) {
					rightRebind.push_back(b);
				}
			}

			// Determine which side the original contour was on
			if (contour->contourPoint.x < 0) {
				rebind = leftRebind;
				leftSide = true;
			}
			else {
				rebind = rightRebind;
				rightSide = true;
			}
			break; // once we find the match, we can exit
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
		left.assign(rebind.begin(),rebind.begin() + index - 1);
		// right
		right.assign(rebind.begin() + index + 1, rebind.end());
	}
	// rebind the contour points
	if (node->addBranch) {
		leftBranchingPointIndex = (left.size()/division);
		rightBranchingPointIndex = index + (right.size()/division);
		// interpolate points in between
		if (leftSide) {  // new branch added on the left
			int rightMostIndex = rebind.size() - 1;
			for (int i = 0; i < leftBranchingPointIndex; i++) {   
				rebind[i]->parentNode = node->parent->parent;
				rebind[i]->childNode = node->parent;
				rebind[i]->t = ((float)i / leftBranchingPointIndex);
				if (rebind[i]->t == 0 && node->parent->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent->parent;
					rebind[i]->childNode = node->parent->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {    
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = (float)(i - leftBranchingPointIndex) / (index - leftBranchingPointIndex);
				if (rebind[i]->t == 0 && node->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent;
					rebind[i]->childNode = node->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = index + 1; i < rightBranchingPointIndex; i++) {
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = 1 - ((float)(i - index) / (rightBranchingPointIndex - index));
				if (rebind[i]->t == 0 && node->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent;
					rebind[i]->childNode = node->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = rightBranchingPointIndex + 1; i < rebind.size(); i++) {
				rebind[i]->parentNode = rebind[rightMostIndex]->parentNode;
				rebind[i]->childNode = rebind[rightMostIndex]->childNode;
				rebind[i]->t = (float)(i - rightBranchingPointIndex) / (rebind.size() - rightBranchingPointIndex);
				if (rebind[i]->t == 0 && rebind[rightMostIndex]->parentNode->parent != NULL) {
					rebind[i]->parentNode = rebind[rightMostIndex]->parentNode->parent;
					rebind[i]->childNode = rebind[rightMostIndex]->parentNode;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
		}
		if (rightSide) {    // new branch added on the right
			int leftMostIndex = 0;
			for (int i = 0; i < leftBranchingPointIndex; i++) {   
				rebind[i]->parentNode = rebind[leftMostIndex]->parentNode;
				rebind[i]->childNode = rebind[leftMostIndex]->childNode;
				rebind[i]->t = 1 - ((float)i / leftBranchingPointIndex);
				if (rebind[i]->t == 0 && rebind[leftMostIndex]->parentNode->parent != NULL) {
					rebind[i]->parentNode = rebind[leftMostIndex]->parentNode->parent;
					rebind[i]->childNode = rebind[leftMostIndex]->parentNode;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = leftBranchingPointIndex + 1; i < index; i++) {    
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = ((float)(i- leftBranchingPointIndex) / (index- leftBranchingPointIndex));
				if (rebind[i]->t == 0 && node->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent;
					rebind[i]->childNode = node->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = index + 1; i < rightBranchingPointIndex; i++) {
				rebind[i]->parentNode = node->parent;
				rebind[i]->childNode = node;
				rebind[i]->t = 1 - ((float)(i - index) / (rightBranchingPointIndex - index));
				if (rebind[i]->t == 0 && node->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent;
					rebind[i]->childNode = node->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
			for (int i = rightBranchingPointIndex + 1; i < rebind.size(); i++) {
				rebind[i]->parentNode = node->parent->parent;
				rebind[i]->childNode = node->parent;
				rebind[i]->t = 1 - ((float)(i - rightBranchingPointIndex) / (rebind.size() - rightBranchingPointIndex));
				if (rebind[i]->t == 0 && node->parent->parent->parent != NULL) {
					rebind[i]->parentNode = node->parent->parent->parent;
					rebind[i]->childNode = node->parent->parent;
					rebind[i]->t = 1;
				}
				rebind[i]->closestPoint = rebind[i]->t * rebind[i]->childNode->globalTransformation[3] + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation[3];
				rebind[i]->previousAnimateInverse = glm::inverse(rebind[i]->t * rebind[i]->childNode->globalTransformation + (1 - rebind[i]->t) * rebind[i]->parentNode->globalTransformation);
			}
		}
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

		// reset new branch
		node->addBranch = false;
	}
	else {
		for (SceneNode* child : node->children) {
			rebindContourToNewBranch(child, contour, division, toRebind);
		}
	}
}
*/

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
		if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].childNode == bindings[i + 1].parentNode && bindings[i].t != 1) {
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
		else if (bindings[i].childNode != bindings[i + 1].childNode && bindings[i].parentNode == bindings[i + 1].childNode && bindings[i + 1].t != 1) {
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

std::vector<ContourBinding> SceneNode::addContourPoints(std::vector<ContourBinding>& bindings, std::vector<SceneNode*>& branchingStructure) {
	std::vector<ContourBinding> newBindingSet;
	// arbitrary threshold
	float threshold = 0.1f;
	for (int i = 0; i < bindings.size() - 1; i++) {
		float distance = glm::length(bindings[i + 1].contourPoint - bindings[i].contourPoint);
		if (distance >= threshold) {
			glm::vec3 newPoint = glm::mix(bindings[i].contourPoint, bindings[i + 1].contourPoint, 0.5f);
			// special case: contour points belong to different branch
			if (bindings[i].childNode != bindings[i + 1].childNode) {
				// find younger branch
				ContourBinding neighbor = (getDeeperNode(bindings[i].childNode, bindings[i + 1].childNode) == bindings[i].childNode) ? bindings[i] : bindings[i + 1];
				glm::vec3 neighborParent = neighbor.parentNode->globalTransformation[3];
				glm::vec3 neighborChild = neighbor.childNode->globalTransformation[3];
				float t = neighbor.t / 2.f;
				glm::mat4 previousiInverseAnimationMat = glm::inverse(t * neighbor.childNode->globalTransformation + (1 - t) * neighbor.parentNode->globalTransformation);
				newBindingSet.push_back(bindings[i]);
				newBindingSet.push_back({ neighbor.parentNode, neighbor.childNode, newPoint, t, glm::mix(bindings[i].closestPoint, bindings[i + 1].closestPoint, 0.5f), previousiInverseAnimationMat });
			}
			else {
				glm::vec3 neighborParent = bindings[i].parentNode->globalTransformation[3];
				glm::vec3 neighborChild = bindings[i].childNode->globalTransformation[3];
				float t = (bindings[i].t + bindings[i + 1].t) / 2.f;
				if (t < 0.05f && bindings[i].parentNode->parent != NULL) {     // snap if t is close to 0
					std::cout << t << std::endl;
					glm::vec3 neighborParent = bindings[i].parentNode->parent->globalTransformation[3];
					glm::vec3 neighborChild = bindings[i].parentNode->globalTransformation[3];
					float t = 1.f;
					glm::mat4 previousiInverseAnimationMat = glm::inverse(t * bindings[i].parentNode->globalTransformation + (1 - t) * bindings[i].parentNode->parent->globalTransformation);
					newBindingSet.push_back(bindings[i]);
					newBindingSet.push_back({ bindings[i].parentNode->parent, bindings[i].parentNode, newPoint, t, glm::mix(bindings[i].closestPoint, bindings[i + 1].closestPoint, 0.5f), previousiInverseAnimationMat });
					continue;
				}
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

// relative transformation between frame (global frame)
void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings) {
	// need the previous frame's animation matrix and current frame's animation matrix
	// update the contour point every frame (in the ContourBinding) so that you just apply the matrix to the contour point
	// this is in the "global" frame
	for (auto& binding : bindings) {
		glm::mat4 animatedPosMat = binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation;
		binding.contourPoint = animatedPosMat * binding.previousAnimateInverse * glm::vec4(binding.contourPoint, 1.0f);
		binding.previousAnimateInverse = glm::inverse(binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation);
		binding.closestPoint = binding.t * binding.childNode->globalTransformation[3] + (1 - binding.t) * (binding.parentNode->globalTransformation[3]);
	}
}

//void SceneNode::animationPerFrame(std::vector<ContourBinding>& bindings) {
//	// Open the file (append mode so you don’t overwrite previous frames)
//	std::ofstream file("animated_matrices_no_division_scaling2.txt", std::ios::trunc);
//	if (!file) {
//		std::cerr << "Could not open file for writing.\n";
//		return;
//	}
//	int index = 0;
//	for (auto& binding : bindings) {
//		glm::mat4 matrix = binding.t * binding.childNode->globalTransformation + (1 - binding.t) * binding.parentNode->globalTransformation;
//		// Write matrix to file
//		if (index == 9 && animated) {
//			file << binding.contourPoint.x << " " << binding.contourPoint.y << " " << binding.contourPoint.z << "\n";
//			for (int i = 0; i < 4; ++i) {
//				file << "[ ";
//				for (int j = 0; j < 4; ++j) {
//					file << matrix[i][j] << " ";
//				}
//				file << "]\n";
//			}
//			file << "\n";
//			matrix *= binding.previousAnimateInverse;
//			for (int i = 0; i < 4; ++i) {
//				file << "[ ";
//				for (int j = 0; j < 4; ++j) {
//					file << matrix[i][j] << " ";
//				}
//				file << "]\n";
//			}
//			file << "\n";
//			animated = false;
//		}
//
//		// Update binding
//		glm::mat4 animatedPosMat = binding.t * binding.childNode->globalTransformation +
//			(1 - binding.t) * binding.parentNode->globalTransformation;
//		binding.contourPoint = animatedPosMat * binding.previousAnimateInverse * glm::vec4(binding.contourPoint, 1.0f);
//		binding.previousAnimateInverse = glm::inverse(binding.t * binding.childNode->globalTransformation +
//			(1 - binding.t) * binding.parentNode->globalTransformation);
//		binding.closestPoint = binding.t * binding.childNode->globalTransformation[3] +
//			(1 - binding.t) * (binding.parentNode->globalTransformation[3]);
//		index += 1;
//	}
//
//	file << "-------------------------\n";
//	file.close(); // Close the file after writing
//}


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
