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
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
	root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);

	//root->animate(0);
	//root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
	//root->animationPerFrame(bindings, 0);
	//if (addContour) {
	//	bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
	//	branchingStructure.clear();
	//	accumulateBranchingStructure(root, branchingStructure);
	//}
}

void Simulation::init(const std::string& path, bool isTxt) {
	std::string lower = path;
	std::transform(lower.begin(), lower.end(), lower.begin(),
		[](unsigned char c) { return std::tolower(c); });

	if (isTxt)
	{
		edgeTransforms = SceneNode::extractEdgeTransformsTxt(path);
	}
	else
	{
		edgeTransforms = SceneNode::extractEdgeTransformsToml(path);
	}

	auto parentChildPairs = SceneNode::buildChildrenList(edgeTransforms);

	root = SceneNode::createBranchingStructure(
		0, parentChildPairs, edgeTransforms
	);

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		branchGeometry
	);

	auto contour = root->generateInitialContourControlPoints(root);  // generate contour points associated to root and leaf nodes
	contour = root->midPoints(contour);

	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	int idx = 0;
	root->labelBranches(root, pairs, idx);

	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	root->getBranches(root, newPairs);

	auto grouped = root->contourCatmullRomGrouped(contour, 25, pairs);

	bindings = root->bindInterpolatedContourToBranches(grouped);
}

void Simulation::step(float dt) {
	branchGeometry.verts.clear();
	branchGeometry.cols.clear();
	branchGeometry.indices.clear();

	contourGeometry.verts.clear();
	contourGeometry.cols.clear();

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		branchGeometry
	);

	bindings = root->addContourPoints(bindings);

	root->animationPerFrame(bindings, dt);
	root->calculateNormalDirection(bindings);

	// rebuild contour geometry
	contourGeometry.verts.clear();
	for (auto& b : bindings)
		contourGeometry.verts.push_back(b.contourPoint);
}

void Simulation::updateSimulation(float dt)
{
	// -----------------------------
	// BRANCH UPDATE
	// -----------------------------
	branchGeometry.verts.clear();
	branchGeometry.cols.clear();
	branchGeometry.indices.clear();

	contourGeometry.verts.clear();
	contourGeometry.cols.clear();

	for (auto& b : branchUpdates) {
		b.verts.clear();
		b.cols.clear();
		b.indices.clear();
	}

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		branchGeometry
	);

	// -----------------------------
	// STRUCTURE UPDATE
	// -----------------------------
	branchingStructure.clear();
	accumulateBranchingStructure(root, branchingStructure);

	bindings = root->addContourPoints(bindings);
	bindings = root->addContourPointsLargeBinding(bindings);
}

void Simulation::animateRebuild(float dt) {
	root->animationPerFrame(bindings, dt);
	root->calculateNormalDirection(bindings);
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
	g_pressed = true;
	root->animate(dt);
}


void Simulation::simulateSubdivision(float length, float dt)
{
	while (root->divideBranch(root, 0.025f, 2.f, false)) {
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
		simulateGrowth(dt);
		updateSimulation(dt);
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

void Simulation::handleSKey(int keyState)
{

	if (keyState == GLFW_PRESS) {

		if (!sPressed) {  // ensures "once per press"
			sPressed = true;

			if (root->divideBranch(root, 0.01f, 2.f, false)) {
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

	if (keyState == GLFW_RELEASE) {
		sPressed = false;
	}
}

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

void Simulation::handleRemoveBranchClick(const glm::vec3& worldPos, bool& mouseClicked)
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
							glm::mat4(1.0f),
							branchGeometry
						);

						root->rebindContourWithMergedBranch(root, bindings);
						resetBool(root);

						return; // important: stop after merge
					}
				}
			}
		}
	}

	mouseClicked = false;
}

void Simulation::handleAddBranchClick(const glm::vec3& worldPos, bool& mouseClicked)
{

	if (mouseClicked) {
		ContourBinding* c = nullptr;
		for (int i = 0; i < bindings.size(); i++) {
			// clicked on a point
			if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-02) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-02)) {
				c = &bindings[i];
				break;
			}
		}
		if (c == nullptr) {
			//std::cout << "contour point not clicked" << std::endl;
		}
		// don't want to add a new branch relative to the contour point that is bound to leaf node (don't want a vertical branch) -> might not need?
		else if (!(c->childNode->children.empty())) {
			printf("%f %f %f\n",worldPos.x,worldPos.y,worldPos.z);
			//if (root->divideBranchMinDistance(root, c))
			ContourBinding pointToBreak = root->findBestBinding(root, c->contourPoint);
			if (root->splitAtBinding(&pointToBreak))
				splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);
			// add new branch
			SceneNode* newNode = root->addNewBranch(root, c, maxID);
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
			root->rebindToNewBranch(newNode, c, bindings, 0.1f);
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
			for (int i = 0; i <= bindings.size() / 2; i++) {
				firstHalf.push_back(&bindings[i]);
			}
			std::vector<ContourBinding*> secondHalf;
			for (int i = bindings.size() - 1; i >= bindings.size() / 2; i--) {
				secondHalf.push_back(&bindings[i]);
			}
			//std::cout << "points: " << bindings.size() << std::endl;
			root->reorganizeChildrenLeft(root);
			std::vector<std::pair<int, BranchKey>> mismatchLeft = root->findMisorientedContourIndices(root, firstHalf);
			//std::cout << "mismatch indices size left: " << mismatchLeft.size() << std::endl;
			root->reorganizeChildrenRight(root);
			std::vector<std::pair<int, BranchKey>> mismatchRight = root->findMisorientedContourIndices(root, secondHalf);
			//std::cout << "mismatch indices size right: " << mismatchRight.size() << std::endl;
			resetBool(root);

			// update transformation for prev frame (delta time = 0)
			//root->animate(0);
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
			//root->animationPerFrame(bindings, 0);
			/*root->printBranches(root);
			std::cout << "------------" << std::endl;*/
			mouseClicked = false;
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
		auto folderPath = "geometry_data";

		// Create directory if it doesn't exist
		if (!std::filesystem::exists(folderPath)) {
			std::filesystem::create_directory(folderPath);
		}

		// Timestamped filename
		auto now = std::time(nullptr);
		auto tm = *std::localtime(&now);

		std::ostringstream oss;
		oss << folderPath;
		oss << "/geometry_";
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

void Simulation::stepHeadless(float dt, float length)
{
	if (!subdivisionDone) {
		simulateSubdivision(length, dt);
		subdivisionDone = true;
	}
	simulateGrowth(dt);
	updateSimulation(dt);
	if (g_pressed) animateRebuild(dt);
	rebuildContourGeometry();
	rebuildDebugGeometry();
}
