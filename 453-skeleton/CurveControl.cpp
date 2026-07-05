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

void Simulation::splitBranch(SceneNode* root, CPU_Geometry& branchGeometry, std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs, std::vector<std::pair<SceneNode*, SceneNode*>>& newPairs, int index, std::vector<SceneNode*>& branchingStructure, bool addContour)
{
	pairs.clear();
	root->labelBranches(root, pairs, index);
	newPairs.clear();
	index = 0;
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
	root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
	//if (addContour) {
	//	bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
	//	branchingStructure.clear();
	//	accumulateBranchingStructure(root, branchingStructure);
	//}
}

float Simulation::init(const std::string& path, bool isTxt, const std::string& newBranch, const std::string& sim) {
	std::string lower = path;
	std::transform(lower.begin(), lower.end(), lower.begin(),
		[](unsigned char c) { return std::tolower(c); });

	// use absolute path for optimizer
	// when python runs simulation, the working directory is where the python code is, not here
	NewBranchConfig::instance().load(newBranch);
	SimulationConfig::instance().load(sim);

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

	auto contour = root->generateInitialContourControlPoints(root);  // generate contour points associated to root and leaf nodes
	contour = root->midPoints(contour);

	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	int idx = 0;
	root->labelBranches(root, pairs, idx);

	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	root->getBranches(root, newPairs);

	auto grouped = root->contourCatmullRomGrouped(contour, pairs);

	bindings = root->bindInterpolatedContourToBranches(grouped);

	return root->deltaTime;
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

void Simulation::animateRebuild(float dt) {
	root->animationPerFrame(bindings, dt);
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

	for (size_t i = 0; i < contourGeometry.verts.size(); i++) {
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
	//g_pressed = true;
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
	//while (root->divideBranch(root, 0.02f, 2.f, false)) {
	if (root->divideBranch(root, false)) {
		splitBranch(
			root,
			branchGeometry,
			bindings,
			pairs,
			newPairs,
			index = 0,
			branchingStructure,
			true
		);
		resetBool(root);
		//simulateGrowth(dt);
		//updateSimulation(dt);
		//if (g_pressed) animateRebuild(dt);
	}
}

//void Simulation::handleSKey()
//{
//	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);
//
//	if (state == GLFW_PRESS) {
//
//		if (root->divideBranch(root, 0.01f, 2.f, false)) {
//
//			splitBranch(
//				root,
//				branchGeometry,
//				bindings,
//				pairs,
//				newPairs,
//				index = 0,
//				branchingStructure,
//				true
//			);
//
//			resetBool(root);
//		}
//	}
//
//	if (state == GLFW_RELEASE) {
//		sPressed = false;
//	}
//}

void Simulation::handleSKey()
{
	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);

	if (state == GLFW_PRESS) {

		if (!sPressed) {  // ensures "once per press"
			sPressed = true;

			if (root->divideBranch(root, false)) {
				splitBranch(
					root,
					branchGeometry,
					bindings,
					pairs,
					newPairs,
					index = 0,
					branchingStructure,
					true
				);
				resetBool(root);
			}
		}
	}

	if (state == GLFW_RELEASE) {
		sPressed = false;
	}
}

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

void Simulation::handleGKey(float dt)
{
	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_G);

	if (state == GLFW_PRESS)
	{
		g_pressed = true;
		root->animate(dt);
	}

	// NEED THIS, else code thinks G is always pressed
	if (state == GLFW_RELEASE) {
		g_pressed = false;
	}
}

void Simulation::pressGKey(float dt, int state) {
	if (state == GLFW_PRESS)
	{
		g_pressed = true;
		simulateGrowth(dt);
	}
}

void Simulation::releaseGKey(int state) {
	if (state == GLFW_RELEASE)
		g_pressed = false;
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

void Simulation::handleAddBranchClick(const glm::vec3& worldPos, bool mouseClicked, bool perpendicular)
{
	if (mouseClicked) {
		ContourBinding* c = nullptr;
		//for (int i = 0; i < bindings.size(); i++) {
		//	if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-02) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-02)) {
		//		c = &bindings[i];
		//		break;
		//	}
		//}
		float xDifference = FLT_MAX;
		float yDifference = FLT_MAX;
		for (int i = 0; i < bindings.size(); i++) {
			if ((abs(worldPos.x - bindings[i].contourPoint.x) <= xDifference) && (abs(worldPos.y - bindings[i].contourPoint.y) <= yDifference)) {
				xDifference = abs(worldPos.x - bindings[i].contourPoint.x);
				yDifference = abs(worldPos.y - bindings[i].contourPoint.y);
				c = &bindings[i];
			}
		}
		if (c == nullptr) {
			std::cout << "contour point not clicked" << std::endl;
		}
		else {
			ContourBinding pointToBreak;
			if (perpendicular)
				pointToBreak = root->findBestBindingPerpendicular(root, c->contourPoint);
			else
				pointToBreak = root->findBestBinding(root, c->contourPoint);
			if (root->splitAtBinding(&pointToBreak))
				splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);
			// add new branch
			SceneNode* newNode = root->addNewBranch(root, c, maxID);
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->rebindToNewBranch(newNode, c, bindings);
			maxID = root->getMaxID(root);  // update maxID after you add a new branch to reflect lastest ID
			
			//// add new contour point to split node (dont do this anymore)
			//bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
			//branchingStructure.clear();
			//accumulateBranchingStructure(root, branchingStructure);
			
			// reorganize children (leftmost to rightmost)
			index = 0;
			pairs.clear();
			root->labelBranches(root, pairs, index);
			std::vector<ContourBinding*> firstHalf;
			for (int i = 1; i <= bindings.size() / 2; i++) {
				firstHalf.push_back(&bindings[i]);
			}
			std::vector<ContourBinding*> secondHalf;
			for (int i = bindings.size() - 2; i >= bindings.size() / 2; i--) {
				secondHalf.push_back(&bindings[i]);
			}

			root->reorganizeChildrenLeft(root);
			root->reorganizeChildrenRight(root);
			std::vector<std::pair<int, BranchKey>> mismatchRight = root->findMisorientedContourIndices(root, secondHalf);
			resetBool(root);

			// update transformation for prev frame (delta time = 0)
			root->animate(0);
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f));
			root->animationPerFrame(bindings, 0);
		}
	}
}

void Simulation::handleAKey(float dt)
{
	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_A);

	if (state == GLFW_PRESS)
	{
		std::vector<ContourBinding*> firstHalf;
		for (int i = 0; i <= bindings.size() / 2; i++) {
			firstHalf.push_back(&bindings[i]);
		}
		std::vector<ContourBinding*> secondHalf;
		for (int i = bindings.size() - 1; i >= bindings.size() / 2; i--) {
			secondHalf.push_back(&bindings[i]);
		}
		root->reorganizeChildrenLeft(root);
		std::vector<std::pair<int, BranchKey>> mismatchLeft = root->findMisorientedContourIndices(root, firstHalf);
		root->reorganizeChildrenRight(root);
		std::vector<std::pair<int, BranchKey>> mismatchRight = root->findMisorientedContourIndices(root, secondHalf);
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

void Simulation::screenshot(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
		//taking screenshot
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		saveScreenshot(width, height);
		//screenshotRequested = false;
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
	HoldG,
	Done
};

ScriptPhase scriptPhase = ScriptPhase::Idle;
float phaseElapsed = 0.0f;
const float G_HOLD_DURATION = 1.0f; 

void Simulation::simulationInstructions(float dt) {
	switch (scriptPhase) {
	case ScriptPhase::Idle:
		scriptPhase = ScriptPhase::PressS;
		break;

	case ScriptPhase::PressS:
		pressSKey(GLFW_PRESS);
		releaseSkey(GLFW_RELEASE);
		scriptPhase = ScriptPhase::AddBranch;
		break;

	case ScriptPhase::AddBranch:
		handleAddBranchClick(glm::vec3(0.054f, 0.58057f, 0.0f), true, false);
		pressGKey(dt, GLFW_PRESS);   // g_pressed = true, first growth step
		phaseElapsed = 0.0f;
		scriptPhase = ScriptPhase::HoldG;
		break;

	case ScriptPhase::HoldG:
		simulateGrowth(dt);
		animateRebuild(dt);
		phaseElapsed += dt;
		if (phaseElapsed >= G_HOLD_DURATION) {
			releaseGKey(GLFW_RELEASE);
			scriptPhase = ScriptPhase::Done;
		}
		break;

	case ScriptPhase::Done:
		break;
	}
}

void Simulation::stepHeadless(float dt, float length)
{
	if (!subdivisionDone) {
		simulateSubdivision();
		subdivisionDone = true;
	}
	simulateGrowth(dt);
	updateSimulation();
	if (g_pressed) animateRebuild(dt);
	rebuildContourGeometry();
	rebuildDebugGeometry();
}
