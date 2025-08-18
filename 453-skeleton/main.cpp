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

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float>> edgeTransformations = SceneNode::extractEdgeTransforms("D:\\Program\\C++\\NewPhytologist2017\\articulated-structure\\plyFile\\transform_matrices9.txt");
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
	int index = 0;
	root->labelBranches(root, pairs, index);
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourCatmullRomGrouped(contour, 8, pairs);
	std::vector<ContourBinding> bindings = root->bindInterpolatedContourToBranches(groupedContour);  
	// DEBUGGING PURPOSES
	CPU_Geometry mappingLines;

	// camera setup
	glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));
	glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.f / 800.f, 0.1f, 100.f);
	glm::mat4 viewProj = proj * view;
	glUseProgram(shader);
	glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));
	float lastTime = glfwGetTime();

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
		std::vector<std::pair<SceneNode*, SceneNode*>> p;
		for (const auto& tup : pairs) {
			p.emplace_back(std::get<0>(tup), std::get<1>(tup));
		}
		root->interpolateBranchTransforms(p, branchUpdates);

		// add contour point if necessary and bind
		bindings = root->addContourPoints(bindings);
		root->animationPerFrame(bindings);

		mappingLines.verts.clear();
		mappingLines.indices.clear();

		// UPDATE ONCE BRANCHES INTERPOLATE
		int i = 0;
		for (const auto& binding : bindings) {
			int startIdx = mappingLines.verts.size();
			mappingLines.verts.push_back(binding.contourPoint);
			mappingLines.verts.push_back(binding.closestPoint);
			mappingLines.cols.push_back(glm::vec3(0.5f, 0.f, 0.5f));
			mappingLines.cols.push_back(glm::vec3(0.5f, 0.f, 0.5f));
			mappingLines.indices.push_back(startIdx);     // from contour
			mappingLines.indices.push_back(startIdx + 1); // to closest branch point
			++i;
		}

		// interpolate contour points using catmullrom here
		for (int i = 0; i < bindings.size(); i++) {
			contourGeometry.verts.push_back(bindings[i].contourPoint);
		}
		for (int i = 0; i < contourGeometry.verts.size(); i++) {
			contourGeometry.cols.push_back(glm::vec3(1.f, 0.f, 0.f));
		}

		glPointSize(5);
		glLineWidth(3.0f); // Set line width to 2 pixels
		// Branch
		updateBuffers(branchGeometry.verts, branchGeometry.cols, branchGeometry.indices);
		glBindVertexArray(vao);
		//glDrawArrays(GL_POINTS, 0, branchGeometry.verts.size());
		//glDrawElements(GL_LINES, branchGeometry.indices.size(), GL_UNSIGNED_INT, 0);

		// Interpolated branch
		for (int i = 0; i < branchUpdates.size(); i++) {
			updateBuffers(branchUpdates[i].verts, branchUpdates[i].cols, branchUpdates[i].indices);
			//glDrawArrays(GL_POINTS, 0, branchUpdates[i].verts.size());
			glDrawArrays(GL_LINE_STRIP, 0, branchUpdates[i].verts.size());
			//glDrawElements(GL_LINES, branchUpdates.indices.size(), GL_UNSIGNED_INT, 0);
		}

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

