#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "Geometry.h"
#include "SceneNode.h"
#include "ShaderLoader.h"
#include <tuple>
#include <random>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem> // C++17 or later
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

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


static bool aKeyPressedLastFrame = false;

void accumulateBranchingStructure(SceneNode* root, std::vector<SceneNode*>& branchingStructure) {
	branchingStructure.push_back(root);
	for (SceneNode* child : root->children) {
		accumulateBranchingStructure(child, branchingStructure);
	}
}

void resetBool(SceneNode* root) {
	root->addBranch = false;
	root->midBranch = false;
	root->trackOriginalBranch = false;
	root->toMerge = false;
	for (SceneNode* child : root->children) {
		resetBool(child);
	}
}

float computeMainAxisLength(SceneNode* root) {
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

// DEBUGGING 
void printVectorOfPairs(const std::vector<std::pair<glm::vec3, glm::vec3>>& vec) {
	for (const auto& pair : vec) {
		std::cout << "Pair: " << std::endl;
		std::cout << "  First (vec3): (" << pair.first.x << ", " << pair.first.y << ", " << pair.first.z << ")" << std::endl;
		std::cout << "  Second (vec3): (" << pair.second.x << ", " << pair.second.y << ", " << pair.second.z << ")" << std::endl;
	}
}
void fillMappingGeometry(
	const std::vector<std::pair<glm::vec3, glm::vec3>>& mappings,
	CPU_Geometry& mappingLines
) {
	for (const auto& [contourPoint, closestPoint] : mappings) {
		// add both vertices (pair)
		mappingLines.verts.push_back(contourPoint);
		mappingLines.verts.push_back(closestPoint);

		// indices for the line
		int startIndex = mappingLines.verts.size() - 2;
		mappingLines.indices.push_back(startIndex);
		mappingLines.indices.push_back(startIndex + 1);
	}
}

struct SharedState {
	SceneNode* rootNode = nullptr;
	glm::mat4 viewMatrix;
	glm::mat4 projMatrix;
	//std::vector<glm::vec3> contour;
	//CPU_Geometry geom;
};

SharedState gSharedState;
bool clicked;
glm::vec3 worldPos;

//void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
//	int width, height;
//	glfwGetWindowSize(window, &width, &height);
//
//	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
//		clicked = true;
//
//		double xpos, ypos;
//		glfwGetCursorPos(window, &xpos, &ypos);
//
//		float normalizedX = -1.0f + 2.0f * float(xpos) / width;
//		float normalizedY = 1.0f - 2.0f * float(ypos) / height;
//
//		glm::vec4 clipCoords = glm::vec4(normalizedX, normalizedY, -1.0f, 1.0f); 
//
//		glm::mat4 invProj = glm::inverse(gSharedState.projMatrix);
//		glm::vec4 viewCoords = invProj * clipCoords;
//		viewCoords /= viewCoords.w;
//
//		glm::mat4 invView = glm::inverse(gSharedState.viewMatrix);
//		glm::vec4 worldCoords = invView * viewCoords;
//		worldCoords /= worldCoords.w;
//		worldCoords.z = 0.f;
//
//		std::cout << "World Position: "
//			<< worldCoords.x << ", "
//			<< worldCoords.y << ", "
//			<< worldCoords.z << ", "
//			<< worldCoords.w << std::endl;
//	}
//}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		clicked = true;
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);

		float x = (2.0f * xpos) / width - 1.0f;
		float y = 1.0f - (2.0f * ypos) / height;
		float z = 1.0f;

		glm::vec3 rayNDS(x, y, z);
		glm::vec4 rayClip(rayNDS.x, rayNDS.y, -1.0f, 1.0f);

		glm::vec4 rayEye = glm::inverse(gSharedState.projMatrix) * rayClip;
		rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f); 

		glm::vec3 rayWorld = glm::vec3(glm::inverse(gSharedState.viewMatrix) * rayEye);
		rayWorld = glm::normalize(rayWorld);

		glm::vec3 camPos = glm::vec3(glm::inverse(gSharedState.viewMatrix)[3]);

		float t = -camPos.z / rayWorld.z;
		worldPos = camPos + t * rayWorld;

		std::cout << "World position: (" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")\n";
	}
}


GLuint vao, vboPos, vboColor, ebo;

void setupBuffers() {
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vboPos);
	glBindBuffer(GL_ARRAY_BUFFER, vboPos);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &vboColor);
	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

	glBindVertexArray(0);
}

void updateBuffers(const std::vector<glm::vec3>& verts,
	const std::vector<glm::vec3>& colors,
	const std::vector<unsigned int>& indices) {
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vboPos);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec3), verts.data(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), colors.data(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_DYNAMIC_DRAW);
}

void draw(GLenum primitive, GLsizei vertexCount, GLsizei indexCount) {
	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, vertexCount);
	glDrawElements(primitive, indexCount, GL_UNSIGNED_INT, 0);
}

int main() {
	glfwInit();
	GLFWwindow* window = glfwCreateWindow(800, 800, "Leaf Shape", NULL, NULL);
	glfwMakeContextCurrent(window);
	gladLoadGL();

	glfwSetMouseButtonCallback(window, mouseButtonCallback);

	glEnable(GL_DEPTH_TEST);
	GLuint shader = ShaderLoader("D:/Program/C++/NewPhytologist2017/articulated-structure/articulated-structure/assets/shaders/test.vert", "D:/Program/C++/NewPhytologist2017/articulated-structure/articulated-structure/assets/shaders/test.frag").ID;

	// create and bind VAO and VBO
	setupBuffers();

	CPU_Geometry branchGeometry;
	std::vector<CPU_Geometry> branchUpdates;
	std::vector<SceneNode*> branchingStructure;

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float>> edgeTransformations = SceneNode::extractEdgeTransforms("D:\\Program\\C++\\NewPhytologist2017\\articulated-structure\\plyFile\\transform_matrices7.txt");
	std::vector<std::vector<int>> parentChildPairs = SceneNode::buildChildrenList(edgeTransformations);
	SceneNode* root = SceneNode::createBranchingStructure(0, parentChildPairs, edgeTransformations);
	
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);

	// contour initialization
	CPU_Geometry contourGeometry;
	std::vector<glm::vec3> contour;
	contour = root->generateInitialContourControlPoints(root);
	contour = root->midPoints(contour);
	// branch-contour mapping
	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	int index = 0;
	root->labelBranches(root, pairs, index);
	// catmullrom gives smooth curve, linear gives sharp curve
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourCatmullRomGrouped(contour, 25, pairs);
	//std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourLinearGrouped(contour, 5, pairs);
	std::vector<ContourBinding> bindings = root->bindInterpolatedContourToBranches(groupedContour);
	// DEBUGGING PURPOSES
	CPU_Geometry mappingLines;
	
	// camera setup
	glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));
	glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.f / 800.f, 0.1f, 100.f);
	glm::mat4 viewProj = proj * view;
	gSharedState.viewMatrix = view;
	gSharedState.projMatrix = proj;
	gSharedState.rootNode = root;
	glUseProgram(shader);
	glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));
	float lastTime = glfwGetTime();

	bool sPressed = false;
	int branchCounter = 0;
	bool screenshotRequested = false;
	while (!glfwWindowShouldClose(window)) {
		bool g_pressed = false;
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		float currentTime = glfwGetTime();
		float deltaTime = (currentTime - lastTime) / 10;
		lastTime = currentTime;

		// animate growth
		int state = glfwGetKey(window, GLFW_KEY_G);
		if (state == GLFW_PRESS)
		{
			g_pressed = true;
			root->animate(deltaTime);
		}

		// split branch
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && !sPressed) 
		{
			sPressed = true;
			if (root->divideBranch(root, .1f)) {
				pairs.clear();
				root->labelBranches(root, pairs, index);
				newPairs.clear();
				index = 0;
				root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
				root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
				bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
				branchingStructure.clear();
				accumulateBranchingStructure(root, branchingStructure);
				root->divided = false;
			}
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_RELEASE) {
			sPressed = false;
		}

		// merge branch
		if (clicked) {
			for (int i = 0; i < branchingStructure.size(); i++) {
				// clicked on a branching point
				if ((abs(worldPos.x - branchingStructure[i]->globalTransformation[3].x) <= 0.05) && (abs(worldPos.y - branchingStructure[i]->globalTransformation[3].y) <= 0.05)) {
					if (branchingStructure[i]->parent != NULL && !branchingStructure[i]->children.empty()) {
						glm::vec3 parentBranch = glm::vec3(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]);
						for (int j = 0; j < branchingStructure[i]->children.size(); j++) {
							glm::vec3 childBranch = glm::vec3(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3]);
							float dotProduct = glm::dot(parentBranch, childBranch);
							if (dotProduct == glm::length(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]) * glm::length(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3]) ||
								dotProduct == -glm::length(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]) * glm::length(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3])) {
								root->mergeBranch(root, branchingStructure[i], branchingStructure[i]->parent, branchingStructure[i]->children[j]);
								root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
								root->printStructure(root);
								root->rebindContourWithMergedBranch(root, bindings);
								//for (int k = 0; k < bindings.size(); k++) {
								//	std::cout << glm::to_string(bindings[k].contourPoint) << std::endl;
								//	std::cout << glm::to_string(bindings[k].parentNode->globalTransformation[3]) << std::endl;
								//	std::cout << glm::to_string(bindings[k].childNode->globalTransformation[3]) << std::endl;
								//	std::cout << bindings[k].t << std::endl;
								//}
								resetBool(root);
							}
						}
					}
				}
			}
			clicked = false;
		}

		// add branch
		int currentCKeyState = glfwGetKey(window, GLFW_KEY_A); // only execute once per frame
		if (currentCKeyState == GLFW_PRESS && !aKeyPressedLastFrame)
		{
			float vs[] = {0.994907, 1.237082};
			std::random_device rd;
			std::mt19937 gen(rd()); // random number
			float mainAxisLength = computeMainAxisLength(root);
			std::uniform_real_distribution<float> dist(0.3f, computeMainAxisLength(root));    
			//ContourBinding* c = root->findContourPointToAddBranch(dist(gen), root, bindings);

			//ContourBinding* c = root->findContourPointToAddBranch(vs[branchCounter], root, bindings);
			branchCounter = (branchCounter + 1) % 2;
			
			ContourBinding* c = &bindings[48];
			//ContourBinding* c = &bindings[int(bindings.size()/2 * 1.3)];
			//ContourBinding* c = &bindings[11];
			//ContourBinding* c = &bindings[5];
			//ContourBinding* c = &bindings[3];
			//ContourBinding* c = &bindings[14];
			
			// don't want to add a new branch relative to the contour point that is binded to leaf node (don't want a vertical branch)
			if (!(c->childNode->children.empty())) {
				// storing copy of original branching structure
				std::unordered_map<SceneNode*, SceneNode*> nodeMap;
				SceneNode* originalRoot = SceneNode::cloneSceneNode(root, nullptr, nodeMap);
				std::vector<ContourBinding> originalBindings = bindings;

				root->divideBranchMinDistance(root, c);
				std::vector<size_t> toRebind = root->contourBindingIndicesToRebind(bindings, root);

				if (!toRebind.empty()) {
					pairs.clear();
					root->labelBranches(root, pairs, index);
					newPairs.clear();
					index = 0;
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);

					// now add new branch
					SceneNode* newNode = root->addNewBranch(root, c);
					pairs.clear();
					root->labelBranches(root, pairs, index);
					newPairs.clear();
					index = 0;
					// need to update branch
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					//root->printStructure(root);;
					root->rebindToNewBranch(newNode, toRebind, c, bindings);
					//root->rebindContourToNewBranchIndexBased(newNode, c, 2, toRebind, bindings);
					resetBool(root);
				}
				else {
					printf("no branch added\n");
					root = originalRoot;
					bindings = root->rebindContour(originalBindings, nodeMap);
					resetBool(root);
				}
				root->divided = false;
			}
		}
		// update key state for next frame
		aKeyPressedLastFrame = (currentCKeyState == GLFW_PRESS);
		//root->animate(deltaTime);

		// need to clear geometry before calling update to draw the new positions
		branchGeometry.verts.clear();
		branchGeometry.cols.clear();
		branchGeometry.indices.clear();
		contourGeometry.verts.clear();
		contourGeometry.cols.clear();
		for (int i = 0; i < branchUpdates.size(); i++) {
			branchUpdates[i].verts.clear();
			branchUpdates[i].cols.clear();
			branchUpdates[i].indices.clear();
		}

		// update branch position
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);

		std::vector<std::pair<SceneNode*, SceneNode*>> p;
		for (const auto& tup : pairs) {
			p.emplace_back(std::get<0>(tup), std::get<1>(tup));
		}
		root->interpolateBranchTransforms(p, branchUpdates);

		// add contour point and bind
		branchingStructure.clear();
		accumulateBranchingStructure(root, branchingStructure);
		bindings = root->addContourPoints(bindings);
		//bindings = root->snapContourPoints(bindings);
		if (g_pressed) {
			root->animationPerFrame(bindings);
		}

		mappingLines.verts.clear();
		mappingLines.indices.clear();

		int i = 0;
		for (const auto& binding : bindings) {
			int startIdx = mappingLines.verts.size();
			mappingLines.verts.push_back(binding.contourPoint - glm::vec3(0, 0, 0.02));
			mappingLines.verts.push_back(binding.closestPoint - glm::vec3(0, 0, 0.02));
			mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));
			mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));
			mappingLines.indices.push_back(startIdx);     // from contour
			mappingLines.indices.push_back(startIdx + 1); // to closest branch point
			++i;
		}

		// interpolate contour points
		for (int i = 0; i < bindings.size(); i++) {
			contourGeometry.verts.push_back(bindings[i].contourPoint);
		}

		for (int i = 0; i < contourGeometry.verts.size(); i++) {
			contourGeometry.cols.push_back(glm::vec3(1.f, 0.f, 0.f));
		}

		glPointSize(5);
		glLineWidth(2.0f); // Set line width to 2 pixels
		// Branch
		updateBuffers(branchGeometry.verts, branchGeometry.cols, branchGeometry.indices);
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, branchGeometry.verts.size());
		glDrawElements(GL_LINES, branchGeometry.indices.size(), GL_UNSIGNED_INT, 0);

		//// Interpolated branch
		//for (int i = 0; i < branchUpdates.size(); i++) {
		//	updateBuffers(branchUpdates[i].verts, branchUpdates[i].cols, branchUpdates[i].indices);
		//	glDrawArrays(GL_LINE_STRIP, 0, branchUpdates[i].verts.size());
		//}

		// Contour
		updateBuffers(contourGeometry.verts, contourGeometry.cols, {});
		glDrawArrays(GL_POINTS, 0, contourGeometry.verts.size());
		glDrawArrays(GL_LINE_STRIP, 0, contourGeometry.verts.size());

		// Mapping (DEBUGGING PURPOSES)
		updateBuffers(mappingLines.verts, mappingLines.cols, mappingLines.indices);
		glDrawArrays(GL_POINTS, 0, mappingLines.verts.size());
		draw(GL_LINES, mappingLines.verts.size(), mappingLines.indices.size());

		// Screenshot handling (AFTER rendering, BEFORE buffer swap)
		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
			int width, height;
			glfwGetFramebufferSize(window, &width, &height);
			saveScreenshot(width, height);
			screenshotRequested = false;
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

