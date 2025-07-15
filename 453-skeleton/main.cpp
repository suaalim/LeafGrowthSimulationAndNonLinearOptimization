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

// Outside your loop (global or static variable)
static bool cKeyPressedLastFrame = false;

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
	for (SceneNode* child : root->children) {
		resetBool(child);
	}
}


// DEBUGGING PURPOSES
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

// SharedState.h
struct SharedState {
	SceneNode* rootNode = nullptr;
	glm::mat4 viewMatrix;
	glm::mat4 projMatrix;
	std::vector<glm::vec3> contour;
	CPU_Geometry geom;
};

SharedState gSharedState;
bool clicked;

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		clicked = true;
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);

		if (gSharedState.rootNode) {
			gSharedState.rootNode->handleMouseClick(
				xpos, ypos, 800.f, 800.f,
				gSharedState.viewMatrix,
				gSharedState.projMatrix,
				gSharedState.contour,
				gSharedState.geom
			);
		}
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
	GLFWwindow* window = glfwCreateWindow(800, 800, "Branching Structure", NULL, NULL);
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

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float>> edgeTransformations = SceneNode::extractEdgeTransforms("D:\\Program\\C++\\NewPhytologist2017\\articulated-structure\\plyFile\\transform_matrices7.txt");
	std::vector<std::vector<int>> parentChildPairs = SceneNode::buildChildrenList(edgeTransformations);
	SceneNode* root = SceneNode::createBranchingStructure(0, parentChildPairs, edgeTransformations);
	
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);

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
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourCatmullRomGrouped(contour, 5, pairs);
	//std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourLinearGrouped(contour, 5, pairs);
	std::vector<ContourBinding> bindings = root->bindInterpolatedContourToBranches(groupedContour);
	// DEBUGGING PURPOSES
	CPU_Geometry mappingLines;
	
	// camera setup
	glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));
	glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.f / 800.f, 0.1f, 100.f);
	glm::mat4 viewProj = proj * view;
	glUseProgram(shader);
	glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));
	float lastTime = glfwGetTime();

	int branchCounter = 0;
	while (!glfwWindowShouldClose(window)) {
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// animation
		float currentTime = glfwGetTime();
		float deltaTime = (currentTime - lastTime) / 10;
		lastTime = currentTime;
		int state = glfwGetKey(window, GLFW_KEY_E);
		if (state == GLFW_PRESS)
		{
			root->animate(deltaTime);
		}
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		{
			if (root->divideBranch(root, .5f)) {
				pairs.clear();
				root->labelBranches(root, pairs, index);
				newPairs.clear();
				index = 0;
				root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
				root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
				bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
				accumulateBranchingStructure(root, branchingStructure);
				root->divided = false;
			}
		}

		// only execute once per frame
		int currentCKeyState = glfwGetKey(window, GLFW_KEY_C);
		if (currentCKeyState == GLFW_PRESS && !cKeyPressedLastFrame)
		{
			float vs[] = {0.994907, 1.237082};
			std::random_device rd;
			std::mt19937 gen(rd()); // random number
			std::uniform_real_distribution<float> dist(0.3f, 2.0f);    // should change to distance of the main axis, don't want to add new branch from the root
			ContourBinding* c = root->findContourPointToAddBranch(dist(gen), root, bindings);

			//ContourBinding* c = root->findContourPointToAddBranch(vs[branchCounter], root, bindings);
			//branchCounter = (branchCounter + 1) % 2;
			
			//ContourBinding* c = &bindings[35];
			//ContourBinding* c = &bindings[int(bindings.size()/2 * 1.3)];
			//ContourBinding* c = &bindings[11];
			//ContourBinding* c = &bindings[5];
			//ContourBinding* c = &bindings[3];
			//ContourBinding* c = &bindings[14];
			
			// don't want to add a new branch relative to the contour point that is binded to leaf node (don't want a vertical branch)
			if (!(c->childNode->children.empty() && c->t == 1)) {
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
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);

					// now add new branch
					SceneNode* newNode = root->addNewBranch(root, c);
					pairs.clear();
					root->labelBranches(root, pairs, index);
					newPairs.clear();
					index = 0;
					// need to update branch
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					//root->printStructure(root);;
					root->rebindContourToNewBranchIndexBased(newNode, c, 2, toRebind, bindings);
					resetBool(root);
				}
				else {
					printf("no branch added");
					root = originalRoot;
					bindings = root->rebindContour(originalBindings, nodeMap);
					resetBool(root);
				}
				root->divided = false;
			}
		}
		// update key state for next frame
		cKeyPressedLastFrame = (currentCKeyState == GLFW_PRESS);
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
		root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
		/*root->printMatrix(root);
		std::cout << "----------------------" << std::endl;*/
		//// divide branch
		//if (root->divideBranch(root, 2.0f)) {
		//	pairs.clear();
		//	root->labelBranches(root, pairs, index);
		//	newPairs.clear();
		//	index = 0;
		//	// update branch position
		//	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
		//	root->rebindContour(root, newPairs, index, bindings);
		//	root->divided = false;
		//}
		std::vector<std::pair<SceneNode*, SceneNode*>> p;
		for (const auto& tup : pairs) {
			p.emplace_back(std::get<0>(tup), std::get<1>(tup));
		}
		root->interpolateBranchTransforms(p, branchUpdates);

		// add contour point and bind
		accumulateBranchingStructure(root, branchingStructure);
		bindings = root->addContourPoints(bindings, branchingStructure);
		root->animationPerFrame(bindings);

		mappingLines.verts.clear();
		mappingLines.indices.clear();

		// UPDATE ONCE BRANCHES INTERPOLATE
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

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

