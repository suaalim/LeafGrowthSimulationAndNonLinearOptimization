#include "CurveControl.h"
#include "SceneNode.h"
#include <vector>
#include <glm/glm.hpp>
#include <utility>      
#include <tuple>        
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem> 
#include <glm/gtx/string_cast.hpp>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../thirdparty/stb/stb_image_write.h"
#include "GLDebug.h"

#include "Geometry.h"
#include "Log.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"

#include <imgui.h>
#include <memory>
#include <glm/gtc/type_ptr.hpp>
#include <cstdlib>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/spline.hpp>
#include <map>
#include <numeric>
#include <cmath>
#include <regex>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <functional>
#include "toml.hpp"
#include <unordered_set>
#include <queue>

void Simulation::accumulateBranchingStructure(SceneNode* root, std::vector<SceneNode*>& branchingStructure) {
	branchingStructure.push_back(root);
	for (SceneNode* child : root->children) {
		accumulateBranchingStructure(child, branchingStructure);
	}
}

void Simulation::resetBool(SceneNode* root) {
	root->addBranch = false;
	root->midBranch = false;
	root->trackOriginalBranch = false;
	root->toMerge = false;
	//root->divided = false;
	//root->merged = false;
	for (SceneNode* child : root->children) {
		resetBool(child);
	}
}

float Simulation::computeMainAxisLength(SceneNode* root) {
	if (!root || root->children.empty()) return 0.0f;

	float totalLength = 0.0f;
	SceneNode* current = root;

	while (!current->children.empty()) {
		// Assume first child is the "main" branch
		SceneNode* child = current->children[0];

		glm::vec3 parentPos = glm::vec3(current->globalTransformation[3]);
		glm::vec3 childPos = glm::vec3(child->globalTransformation[3]);
		totalLength += glm::length(childPos - parentPos);

		current = child;
	}

	return totalLength;
}

void Simulation::init(const std::string& path, bool isTxt, const std::string& newBranch, const std::string& sim, const std::string& petiole) {
	std::string lower = path;
	std::transform(lower.begin(), lower.end(), lower.begin(),
		[](unsigned char c) { return std::tolower(c); });

	// use absolute path for optimizer
	// when python runs simulation, the working directory is where the python code is, not here
	NewBranchConfig::instance().load(newBranch);
	SimulationConfig::instance().load(sim);
	PetioleConfig::instance().load(petiole);

	if (isTxt)
	{
		edgeTransforms = FileParser::extractEdgeTransformsTxt(path);
	}
	else
	{
		edgeTransforms = FileParser::extractEdgeTransformsToml(path);
	}

	auto parentChildPairs = SceneNode::buildChildrenList(edgeTransforms);

	root = SceneNode::createBranchingStructure(
		0, parentChildPairs, edgeTransforms
	);

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f)
	);

	root->readNewBranchParameter();
	root->readSimulationParameter();
	root->readPetioleParameter();

	auto contour = root->generateInitialContourControlPoints(root);  // generate contour points associated to root and leaf nodes
	contour = root->midPoints(contour);

	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	int idx = 0;
	root->labelBranches(root, pairs, idx);

	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	root->getBranches(root, newPairs);

	auto grouped = root->contourCatmullRomGrouped(contour, pairs);

	if (root->perpendicularBinding) bindings = root->bindContourToBranches(grouped);
	else bindings = root->bindInterpolatedContourToBranches(grouped);
	bindings = root->addContourPoints(bindings);

	// petiole
	DivideBranchResult result2 = root->divideSingleBranch(root, root->children[0], false, root->mainAxisDivision1, 1);
	if (result2.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result2.petiole, result2.results, index, bindings);
		resetBool(root);
	}
	DivideBranchResult result3 = root->divideSingleBranch(root->children[0], root->children[0]->children[0], false, root->mainAxisDivision2, 4);
	if (result3.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result3.petiole, result3.results, index, bindings);
		resetBool(root);
	}
	DivideBranchResult result4 = root->divideSingleBranch(root->children[0]->children[0], root->children[0]->children[0]->children[0], false, 2.0, 5);
	if (result4.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result4.petiole, result4.results, index, bindings);
		resetBool(root);
	}
	DivideBranchResult result5 = root->divideSingleBranch(root->children[0]->children[0], root->children[0]->children[0]->children[0], false, root->mainAxisDivision3, 1);
	if (result5.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result5.petiole, result5.results, index, bindings);
		resetBool(root);
	}
}

void Simulation::clearGeometry() {
	branchGeometry.verts.clear();
	branchGeometry.cols.clear();
	branchGeometry.indices.clear();

	contourGeometry.verts.clear();
	contourGeometry.cols.clear();

	mappingLines.verts.clear();
	mappingLines.cols.clear();
}

void Simulation::updateSimulation()
{
	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f)
	);
	branchingStructure.clear();
	accumulateBranchingStructure(root, branchingStructure);

	bindings = root->addContourPoints(bindings);
	//bindings = root->addContourPointsLargeBinding(bindings);
}

void Simulation::animateRebuild() {
	root->animationPerFrameBinding(bindings);
	root->calculateNormalDirection(bindings);
}

void Simulation::rebuildBranchGeometry() {
	root->saveBranchGeometry(branchGeometry);
}

void Simulation::rebuildContourGeometry()
{
	contourGeometry.verts.clear();
	contourGeometry.cols.clear();

	contourGeometry.verts.reserve(bindings.size());

	for (const auto& b : bindings) {
		contourGeometry.verts.push_back(b.contourPoint);
	}

	for (size_t i = 0; i < bindings.size(); i++) {
		contourGeometry.cols.push_back(glm::vec3(1.f, 0.f, 0.f));
	}
}

void Simulation::rebuildDebugGeometry()
{
	mappingLines.verts.clear();
	mappingLines.cols.clear();
	mappingLines.indices.clear();

	for (int i = 0; i < bindings.size(); i++) {
		int startIdx = mappingLines.verts.size();

		mappingLines.verts.push_back(
			bindings[i].contourPoint - glm::vec3(0, 0, 0.02f)
		);

		mappingLines.verts.push_back(
			bindings[i].closestPoint - glm::vec3(0, 0, 0.02f)
		);

		mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));
		mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));

		mappingLines.indices.push_back(startIdx);
		mappingLines.indices.push_back(startIdx + 1);
	}
}

void Simulation::simulateGrowth(float dt)
{
	root->animate(dt);
	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f)
	);
}

void Simulation::simulateSubdivision()
{
	DivideBranchResult result = root->divideBranch(root, false);
	if (result.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result.petiole, result.results, index, bindings);
		resetBool(root);
	}
}

void Simulation::handleSKey()
{
	int state;
	if (root->subdivideBranch) {
		state = GLFW_PRESS;
	}
	else {
		state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);
	}

	if (state == GLFW_PRESS) {
		DivideBranchResult result = root->divideBranch(root, false);
		if (result.divided) {
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->rebindContourWithBrokenBranch(root, result.petiole, result.results, index, bindings);
			resetBool(root);
		}
	}

	if (state == GLFW_RELEASE) {
		sPressed = false;
	}
}

//void Simulation::handleSKey()
//{
//	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);
//
//	if (state == GLFW_PRESS) {
//
//		if (!sPressed) {  // ensures "once per press"
//			sPressed = true;
//			DivideBranchResult result = root->divideBranch(root, false);
//			if (result.divided) {
//				pairs.clear();
//				root->labelBranches(root, pairs, index);
//				newPairs.clear();
//				index = 0;
//				root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
//				//root->rebindContourWithBrokenBranch(root->findMidNode(root), newPairs, index, bindings);
//				root->rebindContourWithBrokenBranch(root, result.results, index, bindings);
//				//if (addContour) {
//				//	bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
//				//	branchingStructure.clear();
//				//	accumulateBranchingStructure(root, branchingStructure);
//				//}
//				resetBool(root);
//			}
//		}
//	}
//
//	if (state == GLFW_RELEASE) {
//		sPressed = false;
//	}
//}

void Simulation::pressSKey(int state)
{
	if (state == GLFW_PRESS) {
		if (!sPressed) {  
			sPressed = true;
			simulateSubdivision();
		}
	}
}

void Simulation::releaseSkey(int state) {
	if (state == GLFW_RELEASE) {
		sPressed = false;
	}
}

//void Simulation::handleGKey(float dt)
//{
//	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_G);
//
//	if (state == GLFW_PRESS) {
//
//		if (!g_pressed) {  // ensures "once per press"
//			g_pressed = true;
//
//			root->animate(dt);
//		}
//	}
//
//	if (state == GLFW_RELEASE) {
//		g_pressed = false;
//	}
//}

void Simulation::handleGKey()
{
	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_G);

	if (state == GLFW_PRESS)
	{
		g_pressed = true;
		root->animate(root->deltaTime);
	}

	if (state == GLFW_RELEASE) {
		g_pressed = false;
	}
}

void Simulation::pressGKey(int state) {
	if (state == GLFW_PRESS)
	{
		g_pressed = true;
		simulateGrowth(root->deltaTime);
	}
}

void Simulation::releaseGKey(int state) {
	if (state == GLFW_RELEASE)
		g_pressed = false;
}

void Simulation::handleGetContourInformation(const glm::vec3& worldPos, bool mouseClicked) {
	if (mouseClicked) {
		ContourBinding* c = nullptr;
		int index;
		for (int i = 0; i < bindings.size(); i++) {
			if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-03) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-03)) {
				c = &bindings[i];
				index = i;
				break;
			}
		}
		if (c == nullptr) {
			std::cout << "contour point not clicked" << std::endl;
		}
		else {
			std::cout << "i: " << index << std::endl;
			std::cout << "t: " << c->t << std::endl;
			std::cout << "blend turned on?: " << c->blend << std::endl;
			std::cout << "branching node marker?: " << c->branchingNodeMarker << std::endl;
			std::cout << "blend begin: " << c->blendRegionBegining << std::endl;
			std::cout << "blend end: " << c->blendRegionEnd << std::endl;
			std::cout << "parent node: " << glm::vec3(c->parentNode->globalTransformation[3]) << std::endl;
			std::cout << "child node: " << glm::vec3(c->childNode->globalTransformation[3]) << std::endl;
			printMat4((c->blending) * c->childNode->marginTransformation + (1 - (c->blending)) * c->parentNode->marginTransformation * glm::toMat4(c->childNode->localRotation));
		}
	}
}

void Simulation::handleRemoveBranchClick(const glm::vec3& worldPos, bool mouseClicked)
{
	if (!mouseClicked) return; // or remove this flag entirely if moved outside

	for (int i = 0; i < branchingStructure.size(); i++)
	{
		SceneNode* node = branchingStructure[i];

		glm::vec3 nodePos = glm::vec3(node->globalTransformation[3]);

		// click proximity check
		if (glm::abs(worldPos.x - nodePos.x) <= 0.05f &&
			glm::abs(worldPos.y - nodePos.y) <= 0.05f)
		{
			if (node->parent != nullptr && !node->children.empty())
			{
				glm::vec3 parentBranch =
					glm::vec3(node->globalTransformation[3] -
						node->parent->globalTransformation[3]);

				for (SceneNode* child : node->children)
				{
					glm::vec3 childBranch =
						glm::vec3(child->globalTransformation[3] -
							node->globalTransformation[3]);

					float dotProduct = glm::dot(parentBranch, childBranch);

					float parentLen = glm::length(
						node->globalTransformation[3] -
						node->parent->globalTransformation[3]);

					float childLen = glm::length(
						child->globalTransformation[3] -
						node->globalTransformation[3]);

					float eps = 1e-5f;

					if (glm::abs(dotProduct - parentLen * childLen) < eps ||
						glm::abs(dotProduct + parentLen * childLen) < eps)
					{
						root->mergeBranch(
							root,
							node,
							node->parent,
							child
						);

						root->updateBranch(
							glm::mat4(1.0f),
							glm::mat4(1.0f),
							glm::mat4(1.0f),
							glm::mat4(1.0f)
						);

						root->rebindContourWithMergedBranch(root, bindings);
						resetBool(root);

						return; // important: stop after merge
					}
				}
			}
		}
	}
} 

// do not use this anymore
void Simulation::handleAddBranchClick(const glm::vec3& worldPos, bool mouseClicked)
{
	if (mouseClicked) {
		ContourBinding* c = nullptr;
		for (int i = 0; i < bindings.size(); i++) {
			if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-03) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-03)) {
				//std::cout << i << std::endl;
				c = &bindings[i];
				break;
			}
		}
		if (c == nullptr) {
			std::cout << "contour point not clicked" << std::endl;
		}
		else {
			processBranchAddition(c);
		}
	}
}

void Simulation::processBranchAddition(ContourBinding* c) {
	ContourBinding pointToBreak;
	if (root->perpendicularBranch)
		pointToBreak = root->findBestBindingPerpendicular(root, c->contourPoint);
	else
		pointToBreak = root->findBestBinding(root, c->contourPoint);

	DivideBranchResult result = root->splitAtBinding(&pointToBreak);
	if (result.divided) {
		pairs.clear();
		root->labelBranches(root, pairs, index);
		newPairs.clear();
		index = 0;
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
		root->rebindContourWithBrokenBranch(root, result.petiole, result.results, index, bindings);
	}

	// add new branch
	SceneNode* newNode = root->addNewBranch(root, c, maxID);
	pairs.clear();
	root->labelBranches(root, pairs, index);
	newPairs.clear();
	index = 0;
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
	root->rebindToNewBranch(newNode, root, c, bindings);
	root->addContourOverride(bindings, newNode);
	maxID = root->getMaxID(root);

	// reorganize children (leftmost to rightmost)
	index = 0;
	pairs.clear();
	root->labelBranches(root, pairs, index);
	std::vector<ContourBinding*> entire;
	for (int i = 0; i < bindings.size(); i++) {
		entire.push_back(&bindings[i]);
	}
	root->calculateRebindingGlobal(entire);

	// petiole
	if (root->petiole) {
		DivideBranchResult result2 = root->divideSingleBranch(newNode->parent, newNode, false, root->newBranchDivision1, 1);
		if (result2.divided) {
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->rebindContourWithBrokenBranch(root, result2.petiole, result2.results, index, bindings);
			resetBool(root);
		}
		DivideBranchResult result3 = root->divideSingleBranch(newNode->parent, newNode, false, root->newBranchDivision2, 2);
		if (result3.divided) {
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->rebindContourWithBrokenBranch(root, result3.petiole, result3.results, index, bindings);
			resetBool(root);
		}
		DivideBranchResult result4 = root->divideSingleBranch(newNode->parent, newNode, false, root->newBranchDivision3, 3);
		if (result4.divided) {
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->rebindContourWithBrokenBranch(root, result4.petiole, result4.results, index, bindings);
			resetBool(root);
		}
	}
	if (root->blend) root->indentationControl(bindings);
	resetBool(root);

	root->animate(0);
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
	if (root->blend) root->animationPerFrameBinding(bindings);
	else root->animationPerFrame(bindings, 0);
}

void Simulation::dynamicallyAddBranch() {
	ContourBinding* c = root->findBranchAdditionPoints(root, bindings, root->newBranchDistance, root->newBranchExclusion);
	if (c == nullptr) {
		//std::cout << "contour point not clicked" << std::endl;
		return;
	}
	glm::vec3 originalPoint = c->contourPoint;
	// need to capture symmetricTarget before processing c since the bindings will be modified
	glm::vec3 symmetricTarget = glm::vec3(-originalPoint.x, originalPoint.y, originalPoint.z); 
	processBranchAddition(c);
	if (root->symmetricBranch) {
		ContourBinding* symmetric = root->findSymmetricBinding(bindings, /*point=*/originalPoint, /*maxSearchDistance=*/-1.0f, /*mirrorZ=*/false);
		if (symmetric != nullptr) {
			processBranchAddition(symmetric);
		}
	}
}

// global rebinding
void Simulation::handleAKey()
{
	int state;
	if (root->rebindEveryFrame) {
		state = GLFW_PRESS;
	}
	else {
		state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_A);
	}

	if (state == GLFW_PRESS)
	{
		std::vector<ContourBinding*> entire;
		for (int i = 0; i < bindings.size(); i++) {
			entire.push_back(&bindings[i]);
		}
		root->reorganizeChildrenLeft(root);
		root->calculateRebindingGlobal(entire);
		if (root->blend) root->indentationControl(bindings);
		resetBool(root);
	}
}

void saveScreenshot(int width, int height) {
	auto folderPath = "screenshots";
	// Create screenshots directory if it doesn't exist
	if (!std::filesystem::exists(folderPath)) {
		std::filesystem::create_directory(folderPath);
	}

	// Generate timestamped filename
	auto now = std::time(nullptr);
	auto tm = *std::localtime(&now);
	std::ostringstream oss;
	oss << folderPath;
	oss << "/screenshot_";
	oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
	oss << ".png";
	std::string filename = oss.str();

	// Allocate buffer for pixel data (3 bytes per pixel: RGB)
	GLsizei nrChannels = 3;
	GLsizei stride = nrChannels * width;
	stride += (stride % 4) ? (4 - stride % 4) : 0; // Align to 4 bytes
	std::vector<unsigned char> buffer(stride * height);

	// Read pixels from framebuffer (reads from BACK buffer)
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());

	// Flip image vertically (OpenGL has origin at bottom-left)
	std::vector<unsigned char> flipped(stride * height);
	for (int row = 0; row < height; ++row) {
		memcpy(&flipped[row * stride],
			&buffer[(height - 1 - row) * stride],
			stride);
	}

	// Save as PNG 
	stbi_write_png(filename.c_str(), width, height, nrChannels, flipped.data(), stride);
	std::cout << "Saved screenshot: " << filename << std::endl;
}

void Simulation::screenshot(GLFWwindow* window, bool automatic) {
	static bool automaticScreenshots = automatic;
	static bool previousPState = false;
	static auto lastScreenshot = std::chrono::steady_clock::now();

	bool currentPState = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;

	// Detect a single P key press
	if (currentPState && !previousPState) {
		automaticScreenshots = !automaticScreenshots;

		// Take an immediate screenshot when enabled
		if (automaticScreenshots) {
			int width, height;
			glfwGetFramebufferSize(window, &width, &height);
			saveScreenshot(width, height);

			lastScreenshot = std::chrono::steady_clock::now();
		}
	}

	previousPState = currentPState;

	// Automatically take screenshots every 3 seconds
	if (automaticScreenshots) {
		auto now = std::chrono::steady_clock::now();

		if (now - lastScreenshot >= std::chrono::seconds(1)) {
			int width, height;
			glfwGetFramebufferSize(window, &width, &height);

			saveScreenshot(width, height);

			lastScreenshot += std::chrono::seconds(3);
		}
	}
}

void Simulation::saveContourGeometry(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {

		// Helper lambda: find which vertex index corresponds to a node's world position
		auto findVertexIndex = [&](SceneNode* node) -> int {
			glm::vec3 pos = glm::vec3(node->globalTransformation[3]);
			for (int i = 0; i < branchGeometry.verts.size(); i++) {
				if (glm::distance(branchGeometry.verts[i], pos) < 1e-4f)
					return i;
			}
			return -1; // not found
			};

		newPairs.clear();
		root->getBranches(root, newPairs);
		auto folderPath = "leaf geometry";

		// Create directory if it doesn't exist
		if (!std::filesystem::exists(folderPath)) {
			std::filesystem::create_directory(folderPath);
		}

		// Timestamped filename
		auto now = std::time(nullptr);
		auto tm = *std::localtime(&now);

		std::ostringstream oss;
		oss << folderPath;
		oss << "/leaf geometry";
		oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
		oss << ".txt";

		std::string filename = oss.str();

		// Open file
		std::ofstream outFile(filename);

		if (!outFile.is_open()) {
			std::cerr << "Failed to open file for writing.\n";
		}
		else {

			// Save contour points
			outFile << "=== Contour Points ===\n";

			for (int i = 0; i < bindings.size(); i++) {
				glm::vec3 p = bindings[i].contourPoint;

				outFile << "Point " << i << ": "
					<< p.x << " "
					<< p.y << " "
					<< p.z << "\n";
			}

			// Save branch vertices
			outFile << "\n=== Branch Vertices ===\n";
			for (int i = 0; i < branchGeometry.verts.size(); i++) {
				glm::vec3 v = branchGeometry.verts[i];
				outFile << "Vertex " << i << ": "
					<< v.x << " " << v.y << " " << v.z << "\n";
			}

			// Save edges using getBranches result -> will break if two nodes are the same points
			outFile << "\n=== Edges (parent -> child) ===\n";
			for (auto& [parent, child] : newPairs) {
				int parentIdx = findVertexIndex(parent);
				int childIdx = findVertexIndex(child);
				if (parentIdx != -1 && childIdx != -1)
					outFile << parentIdx << " -> " << childIdx << "\n";
			}

			outFile.close();

			std::cout << "Saved geometry data: " << filename << std::endl;
		}
	}
}

// state machine (script for simulation instructions)
// each frame does each phase (case)
enum class ScriptPhase {
	Idle,
	PressS,
	AddBranch,
	AddBranch2,
	AddBranch3,
	HoldG,
	Done
};

ScriptPhase scriptPhase = ScriptPhase::Idle;
float phaseElapsed = 0.0f;
const float G_HOLD_DURATION = 1.0f; 

void Simulation::stepHeadless(float length)
{
	if (!subdivisionDone) {
		simulateSubdivision();
		subdivisionDone = true;
	}
	simulateGrowth(root->deltaTime);
	updateSimulation();
	if (g_pressed) animateRebuild();
	rebuildContourGeometry();
	rebuildDebugGeometry();
}

void Simulation::setVisualization() {
	updateMarkerKeys(bindings, 10, contourMarkerKeys);
}

void Simulation::visualization() {
	contourMarkers = buildContourMarkers(contourGeometry, bindings, contourMarkerKeys, 0.03f);
	//std::vector<size_t> markerKeys;
	//if (bindings.size() > 21) markerKeys.push_back(bindings[21].uniqueKey);
	//if (bindings.size() > 11) markerKeys.push_back(bindings[11].uniqueKey);
	//contourMarkers = buildContourMarkers(contourGeometry, bindings, markerKeys, 0.03f);
}
