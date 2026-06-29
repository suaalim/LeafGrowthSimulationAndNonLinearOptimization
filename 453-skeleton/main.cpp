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
#include "Camera.h"
#include "CurveControl.h"
#include <tuple>
#include <random>
#include <ctime>
#include <iomanip>
#include <filesystem>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "toml.hpp"

int counter = 0;

struct SharedState {
	SceneNode* rootNode = nullptr;
	glm::mat4 viewMatrix;
	glm::mat4 projMatrix;
};

SharedState gSharedState;
bool clickedToRemove;
bool clickedToAdd;
bool clickedToAddRight;
glm::vec3 worldPos;
glm::vec3 cameraStart = glm::vec3(0.0f, 0.70f, 5.0f);
glm::vec3 cameraTarget = glm::vec3(0.0f, 0.70f, 0.0f);

Camera* camera = new Camera(cameraStart, cameraTarget);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	if (camera) {
		camera->processScroll(static_cast<float>(yoffset));
	}
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		clickedToAdd = true;
		int width, height;
		glfwGetWindowSize(window, &width, &height);

		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);

		ypos = height - ypos;

		float depth = 0.0f;

		glm::vec3 screenPos = glm::vec3(xpos, ypos, depth);

		glm::vec4 viewport = glm::vec4(0.0f, 0.0f, width, height);
		worldPos = glm::unProject(
			screenPos,
			gSharedState.viewMatrix,
			gSharedState.projMatrix,
			viewport
		);

		//std::cout << "World position: (" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")\n";
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		clickedToRemove = true;
		int width, height;
		glfwGetWindowSize(window, &width, &height);

		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);

		ypos = height - ypos;

		float depth = 0.0f;

		glm::vec3 screenPos = glm::vec3(xpos, ypos, depth);

		glm::vec4 viewport = glm::vec4(0.0f, 0.0f, width, height);
		worldPos = glm::unProject(
			screenPos,
			gSharedState.viewMatrix,
			gSharedState.projMatrix,
			viewport
		);

		std::cout << "World position: (" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")\n";
	}
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	const float moveSpeed = 0.05f;

	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		if (key == GLFW_KEY_UP) {
			camera->moveCamera(glm::vec3(0.0f, moveSpeed, 0.0f));
		}
		else if (key == GLFW_KEY_DOWN) {
			camera->moveCamera(glm::vec3(0.0f, -moveSpeed, 0.0f));
		}
		else if (key == GLFW_KEY_LEFT) {
			camera->moveCamera(glm::vec3(-moveSpeed, 0.0f, 0.0f));
		}
		else if (key == GLFW_KEY_RIGHT) {
			camera->moveCamera(glm::vec3(moveSpeed, 0.0f, 0.0f));
		}
	}
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
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

int main(int argc, char* argv[]) {
	std::string filePath;
	std::string ext;
	bool isTxt = false;
	if (argc < 2) {
		filePath = "D:\\Code\\C++\\NewPhytologist2017\\NonLinearOptimization\\plyFile\\transform_matrices7.txt";
		ext = ".txt";
	}
	else {
		filePath = argv[1];
		std::filesystem::path path(filePath);
		ext = path.extension().string();
	}

	if (ext == ".txt") {
		// old behavior
		glfwInit();
		GLFWwindow* window = glfwCreateWindow(800, 800, "Leaf Shape", NULL, NULL);
		glfwMakeContextCurrent(window);
		gladLoadGL();

		glfwSetMouseButtonCallback(window, mouseButtonCallback);
		glfwSetScrollCallback(window, scroll_callback);
		glfwSetKeyCallback(window, keyCallback);
		glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

		glEnable(GL_DEPTH_TEST);
		GLuint shader = ShaderLoader(
			"D:/Code/C++/NewPhytologist2017/Code/assets/shaders/test.vert",
			"D:/Code/C++/NewPhytologist2017/Code/assets/shaders/test.frag"
		).ID;


		// create and bind VAO and VBO
		setupBuffers();

		Simulation sim;
		//sim.init("D:/Code/C++/NewPhytologist2017/plyFile/transform_matrices7.txt");
		isTxt = true;
		sim.init(filePath, isTxt);

		float lastTime = glfwGetTime();
		bool t= true;
		{
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			int width, height;
			glfwGetWindowSize(window, &width, &height);
			float currentTime = glfwGetTime();
			float deltaTime = (currentTime - lastTime) / 10;
			lastTime = currentTime;

			// set up and update camera
			glm::mat4 view = camera->getViewMatrix();
			//glm::mat4 proj = camera->getProjectionMatrix((float)width / (float)height);
			glm::mat4 proj = camera->getOrthoMatrix((float)width / (float)height);
			glm::mat4 viewProj = proj * view;
			gSharedState.viewMatrix = view;
			gSharedState.projMatrix = proj;
			glUseProgram(shader);
			glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));

			// animate growth
			sim.handleGKey(deltaTime);

			// split branch
			sim.handleSKey(glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S));

			// merge branch
			sim.handleRemoveBranchClick(worldPos, clickedToRemove);

			// add branch based on clicking
			sim.handleAddBranchClick(worldPos, clickedToAdd);

			// rebind evenly at every time step
			sim.handleAKey(deltaTime);

			if (sim.g_pressed) {
				sim.animateRebuild(deltaTime);
			}
			//sim.animateRebuild(deltaTime);

			// need to clear geometry before calling update to draw the new positions
			sim.updateSimulation(deltaTime);
			// deltaTime is usually very small, between 4 to 6e-05

			sim.rebuildDebugGeometry();
			sim.rebuildContourGeometry();

			glPointSize(5);
			glLineWidth(2.0f); // Set line width to 2 pixels
			// Branch
			updateBuffers(sim.branchGeometry.verts, sim.branchGeometry.cols, sim.branchGeometry.indices);
			glBindVertexArray(vao);
			glDrawArrays(GL_POINTS, 0, sim.branchGeometry.verts.size());
			glDrawElements(GL_LINES, sim.branchGeometry.indices.size(), GL_UNSIGNED_INT, 0);

			//// Interpolated branch
			//for (int i = 0; i < branchUpdates.size(); i++) {
			//	updateBuffers(branchUpdates[i].verts, branchUpdates[i].cols, branchUpdates[i].indices);
			//	glDrawArrays(GL_POINTS, 0, branchUpdates[i].verts.size());
			//	glDrawArrays(GL_LINE_STRIP, 0, branchUpdates[i].verts.size());
			//}

			// Contour
			updateBuffers(sim.contourGeometry.verts, sim.contourGeometry.cols, {});
			glDrawArrays(GL_POINTS, 0, sim.contourGeometry.verts.size());
			glDrawArrays(GL_LINE_STRIP, 0, sim.contourGeometry.verts.size());

			// Mapping (DEBUGGING PURPOSES)
			updateBuffers(sim.mappingLines.verts, sim.mappingLines.cols, sim.mappingLines.indices);
			glDrawArrays(GL_POINTS, 0, sim.mappingLines.verts.size());
			draw(GL_LINES, sim.mappingLines.verts.size(), sim.mappingLines.indices.size());

			//Screenshot handling (AFTER rendering, BEFORE buffer swap)
			//saving contour and branch information
			sim.screenshot(window);
			sim.saveContourGeometry(window);

			glfwSwapBuffers(window);
			glfwPollEvents();



			sim.handleSKey(true);
			sim.handleAddBranchClick(glm::vec3(0.065000, 0.667500, 4.900002), t);
		}
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		float currentTime = glfwGetTime();
		float deltaTime = (currentTime - lastTime) / 10;
		lastTime = currentTime;

		// set up and update camera
		glm::mat4 view = camera->getViewMatrix();
		//glm::mat4 proj = camera->getProjectionMatrix((float)width / (float)height);
		glm::mat4 proj = camera->getOrthoMatrix((float)width / (float)height);
		glm::mat4 viewProj = proj * view;
		gSharedState.viewMatrix = view;
		gSharedState.projMatrix = proj;
		glUseProgram(shader);
		glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));

		while (!glfwWindowShouldClose(window)) {
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			int width, height;
			glfwGetWindowSize(window, &width, &height);
			float currentTime = glfwGetTime();
			float deltaTime = (currentTime - lastTime) / 10;
			lastTime = currentTime;

			// set up and update camera
			glm::mat4 view = camera->getViewMatrix();
			//glm::mat4 proj = camera->getProjectionMatrix((float)width / (float)height);
			glm::mat4 proj = camera->getOrthoMatrix((float)width / (float)height);
			glm::mat4 viewProj = proj * view;
			gSharedState.viewMatrix = view;
			gSharedState.projMatrix = proj;
			glUseProgram(shader);
			glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));

			// animate growth
			sim.handleGKey(deltaTime);

			// split branch
			sim.handleSKey(glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S));

			// merge branch
			sim.handleRemoveBranchClick(worldPos, clickedToRemove);

			// add branch based on clicking
			sim.handleAddBranchClick(worldPos, clickedToAdd);

			// rebind evenly at every time step
			sim.handleAKey(deltaTime);

			if (sim.g_pressed) {
				sim.animateRebuild(deltaTime);
			}
			//sim.animateRebuild(deltaTime);

			// need to clear geometry before calling update to draw the new positions
			sim.updateSimulation(deltaTime);
			// deltaTime is usually very small, between 4 to 6e-05

			sim.rebuildDebugGeometry();
			sim.rebuildContourGeometry();

			glPointSize(5);
			glLineWidth(2.0f); // Set line width to 2 pixels
			// Branch
			updateBuffers(sim.branchGeometry.verts, sim.branchGeometry.cols, sim.branchGeometry.indices);
			glBindVertexArray(vao);
			glDrawArrays(GL_POINTS, 0, sim.branchGeometry.verts.size());
			glDrawElements(GL_LINES, sim.branchGeometry.indices.size(), GL_UNSIGNED_INT, 0);

			//// Interpolated branch
			//for (int i = 0; i < branchUpdates.size(); i++) {
			//	updateBuffers(branchUpdates[i].verts, branchUpdates[i].cols, branchUpdates[i].indices);
			//	glDrawArrays(GL_POINTS, 0, branchUpdates[i].verts.size());
			//	glDrawArrays(GL_LINE_STRIP, 0, branchUpdates[i].verts.size());
			//}

			// Contour
			updateBuffers(sim.contourGeometry.verts, sim.contourGeometry.cols, {});
			glDrawArrays(GL_POINTS, 0, sim.contourGeometry.verts.size());
			glDrawArrays(GL_LINE_STRIP, 0, sim.contourGeometry.verts.size());

			// Mapping (DEBUGGING PURPOSES)
			updateBuffers(sim.mappingLines.verts, sim.mappingLines.cols, sim.mappingLines.indices);
			glDrawArrays(GL_POINTS, 0, sim.mappingLines.verts.size());
			draw(GL_LINES, sim.mappingLines.verts.size(), sim.mappingLines.indices.size());

			//Screenshot handling (AFTER rendering, BEFORE buffer swap)
			//saving contour and branch information
			sim.screenshot(window);
			sim.saveContourGeometry(window);

			glfwSwapBuffers(window);
			glfwPollEvents();
		}

		glfwTerminate();
		return 0;
	}
	else if (ext == ".toml") {
		// old behavior
		glfwInit();
		GLFWwindow* window = glfwCreateWindow(800, 800, "Leaf Shape", NULL, NULL);
		glfwMakeContextCurrent(window);
		gladLoadGL();
		glfwSetMouseButtonCallback(window, mouseButtonCallback);
		glfwSetScrollCallback(window, scroll_callback);
		glfwSetKeyCallback(window, keyCallback);
		glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
		glEnable(GL_DEPTH_TEST);
		GLuint shader = ShaderLoader(
			"D:/Code/C++/NewPhytologist2017/Code/assets/shaders/test.vert",
			"D:/Code/C++/NewPhytologist2017/Code/assets/shaders/test.frag"
		).ID;
		setupBuffers();
		Simulation sim;
		sim.init(filePath, isTxt);
		float lastTime = glfwGetTime();

		//Simulation sim;
		//sim.init(filePath, isTxt);

		//auto startTime = std::chrono::high_resolution_clock::now();
		//auto lastTime = startTime;

		//const float maxDuration = 0.1f; // seconds
		//float accumulatedTime = 0.0f;
		//while (accumulatedTime < maxDuration) {
		float targetTime = 0.3f;      // seconds
		float elapsedTime = 0.0f;
		float growthSinceLastSubdivision = 0.0f;
		while (elapsedTime < targetTime) {
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			int width, height;
			glfwGetWindowSize(window, &width, &height);
			float currentTime = glfwGetTime();
			float deltaTime = (currentTime - lastTime) / 10;
			lastTime = currentTime;
			elapsedTime += deltaTime;
			// set up and update camera
			glm::mat4 view = camera->getViewMatrix();
			glm::mat4 proj = camera->getOrthoMatrix((float)width / (float)height);
			glm::mat4 viewProj = proj * view;
			gSharedState.viewMatrix = view;
			gSharedState.projMatrix = proj;
			glUseProgram(shader);
			glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, GL_FALSE, glm::value_ptr(viewProj));

			//auto currentTime = std::chrono::high_resolution_clock::now();
			//std::chrono::duration<float> frameElapsed = currentTime - lastTime;
			//float deltaTime = frameElapsed.count();

			//lastTime = currentTime;
			sim.stepHeadless(deltaTime, 1.0f);
			////accumulatedTime += deltaTime;

			glPointSize(5);
			glLineWidth(2.0f); // Set line width to 2 pixels
			// Branch
			updateBuffers(sim.branchGeometry.verts, sim.branchGeometry.cols, sim.branchGeometry.indices);
			glBindVertexArray(vao);
			glDrawArrays(GL_POINTS, 0, sim.branchGeometry.verts.size());
			glDrawElements(GL_LINES, sim.branchGeometry.indices.size(), GL_UNSIGNED_INT, 0);
			updateBuffers(sim.contourGeometry.verts, sim.contourGeometry.cols, {});
			glDrawArrays(GL_POINTS, 0, sim.contourGeometry.verts.size());
			glDrawArrays(GL_LINE_STRIP, 0, sim.contourGeometry.verts.size());
			updateBuffers(sim.mappingLines.verts, sim.mappingLines.cols, sim.mappingLines.indices);
			glDrawArrays(GL_POINTS, 0, sim.mappingLines.verts.size());
			draw(GL_LINES, sim.mappingLines.verts.size(), sim.mappingLines.indices.size());
			glfwSwapBuffers(window);
			glfwPollEvents();
		}
		glfwTerminate();
		//}
		////for (int i = 0; i < sim.contourGeometry.verts.size(); i++) {
		////	glm::vec3 v = glm::vec3(sim.contourGeometry.verts[i]);
		////	std::cout << v.x << " " << v.y << " " << v.z << std::endl;
		////}

		// generate toml file
		toml::array verts_array;
		for (size_t i = 0; i < sim.contourGeometry.verts.size(); i++) {
			glm::vec3 v = glm::vec3(sim.contourGeometry.verts[i]);

			toml::table vert;
			vert.insert("x", v.x);
			vert.insert("y", v.y);
			vert.insert("z", v.z);

			verts_array.push_back(vert);
		}
		toml::table out;
		out.insert("verts", verts_array);
		namespace fs = std::filesystem;
		fs::create_directories("toml");
		std::ofstream file("toml/verts.toml");
		file << out;
		return 0;
	}
	else {
		std::cerr << "Unsupported file type: " << ext << "\n";
		return -1;
	}
}
