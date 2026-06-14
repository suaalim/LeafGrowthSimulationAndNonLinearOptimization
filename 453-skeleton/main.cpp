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
#include <sstream>
#include <filesystem> 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "toml.hpp"

//void saveScreenshot(int width, int height) {
//	auto folderPath = "screenshots";
//	// Create screenshots directory if it doesn't exist
//	if (!std::filesystem::exists(folderPath)) {
//		std::filesystem::create_directory(folderPath);
//	}
//
//	// Generate timestamped filename
//	auto now = std::time(nullptr);
//	auto tm = *std::localtime(&now);
//	std::ostringstream oss;
//	oss << folderPath;
//	oss << "/screenshot_";
//	oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
//	oss << ".png";
//	std::string filename = oss.str();
//
//	// Allocate buffer for pixel data (3 bytes per pixel: RGB)
//	GLsizei nrChannels = 3;
//	GLsizei stride = nrChannels * width;
//	stride += (stride % 4) ? (4 - stride % 4) : 0; // Align to 4 bytes
//	std::vector<unsigned char> buffer(stride * height);
//
//	// Read pixels from framebuffer (reads from BACK buffer)
//	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
//
//	// Flip image vertically (OpenGL has origin at bottom-left)
//	std::vector<unsigned char> flipped(stride * height);
//	for (int row = 0; row < height; ++row) {
//		memcpy(&flipped[row * stride],
//			&buffer[(height - 1 - row) * stride],
//			stride);
//	}
//
//	// Save as PNG 
//	stbi_write_png(filename.c_str(), width, height, nrChannels, flipped.data(), stride);
//	std::cout << "Saved screenshot: " << filename << std::endl;
//}

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
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <config_file>\n";
		return -1;
	}
	std::string filePath = argv[1];

	std::filesystem::path path(filePath);
	std::string ext = path.extension().string();
	bool isTxt = false;

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
		bool sPressed = false;
		bool aKeyPressedLastFrame = false;
		bool tKeyPressedLastFrame = false;
		int branchCounter = 0;
		bool screenshotRequested = false;
		bool bidirectionalGrowth = false;

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
			sim.handleSKey();

			// merge branch
			sim.handleRemoveBranchClick(worldPos, clickedToRemove);

			// add branch based on clicking
			sim.handleAddBranchClick(worldPos, clickedToAdd);

			// need to clear geometry before calling update to draw the new positions
			sim.updateSimulation(deltaTime);
			// deltaTime is usually very small, between 4 to 6e-05

			if (sim.g_pressed) {
				sim.animateRebuild(deltaTime);
			}

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

			// Screenshot handling (AFTER rendering, BEFORE buffer swap)
			// saving contour and branch information
			//if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
			//	////taking screenshot
			//	//int width, height;
			//	//glfwGetFramebufferSize(window, &width, &height);
			//	//saveScreenshot(width, height);
			//	//screenshotRequested = false;

			//	// Helper lambda: find which vertex index corresponds to a node's world position
			//	auto findVertexIndex = [&](SceneNode* node) -> int {
			//		glm::vec3 pos = glm::vec3(node->globalTransformation[3]);
			//		for (int i = 0; i < branchGeometry.verts.size(); i++) {
			//			if (glm::distance(branchGeometry.verts[i], pos) < 1e-4f)
			//				return i;
			//		}
			//		return -1; // not found
			//		};

			//	newPairs.clear();
			//	root->getBranches(root, newPairs);
			//	auto folderPath = "geometry_data";

			//	// Create directory if it doesn't exist
			//	if (!std::filesystem::exists(folderPath)) {
			//		std::filesystem::create_directory(folderPath);
			//	}

			//	// Timestamped filename
			//	auto now = std::time(nullptr);
			//	auto tm = *std::localtime(&now);

			//	std::ostringstream oss;
			//	oss << folderPath;
			//	oss << "/geometry_";
			//	oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
			//	oss << ".txt";

			//	std::string filename = oss.str();

			//	// Open file
			//	std::ofstream outFile(filename);

			//	if (!outFile.is_open()) {
			//		std::cerr << "Failed to open file for writing.\n";
			//	}
			//	else {

			//		// Save contour points
			//		outFile << "=== Contour Points ===\n";

			//		for (int i = 0; i < bindings.size(); i++) {
			//			glm::vec3 p = bindings[i].contourPoint;

			//			outFile << "Point " << i << ": "
			//				<< p.x << " "
			//				<< p.y << " "
			//				<< p.z << "\n";
			//		}

			//		// Save branch vertices
			//		outFile << "\n=== Branch Vertices ===\n";
			//		for (int i = 0; i < branchGeometry.verts.size(); i++) {
			//			glm::vec3 v = branchGeometry.verts[i];
			//			outFile << "Vertex " << i << ": "
			//				<< v.x << " " << v.y << " " << v.z << "\n";
			//		}

			//		// Save edges using getBranches result -> will break if two nodes are the same points
			//		outFile << "\n=== Edges (parent -> child) ===\n";
			//		for (auto& [parent, child] : newPairs) {
			//			int parentIdx = findVertexIndex(parent);
			//			int childIdx = findVertexIndex(child);
			//			if (parentIdx != -1 && childIdx != -1)
			//				outFile << parentIdx << " -> " << childIdx << "\n";
			//		}

			//		outFile.close();

			//		std::cout << "Saved geometry data: " << filename << std::endl;
			//	}
			//}

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
		bool sPressed = false;
		bool aKeyPressedLastFrame = false;
		bool tKeyPressedLastFrame = false;
		int branchCounter = 0;
		bool screenshotRequested = false;
		bool bidirectionalGrowth = false;

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

