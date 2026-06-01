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
#include <filesystem> 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h" 

// camera class for viewing pipeline
class Camera {
public:
	// variables used in camera
	glm::vec3 position;
	glm::vec3 target;
	glm::vec3 up;

	// perspective matrix
	float r;             // radius (from target to cursor pos)
	float theta;         // angle controlling vertical movement
	float phi;           // angle controlling horizontal movement
	float fov;			 // field of view

	// orthogonal matrix
	float orthoLeft = -1.0f;
	float orthoRight = 1.0f;
	float orthoBottom = -1.0f;
	float orthoTop = 1.0f;
	float orthoNear = 0.1f;
	float orthoFar = 100.0f;

	Camera(glm::vec3 startPos, glm::vec3 startTarget)
		: position(startPos), target(startTarget), up(0.0f, 1.0f, 0.0f), fov(45.0f), r(glm::length(position - target)), phi(0.0f), theta(0.0f) {
	}

	// we think of the camera moving along a sphere (arcball)
	// so we use spherical coordinates for the sphere
	// update the x, y, z values based on the updated r, theta and phi
	void updateCameraPosition() {
		position.x = r * sin(glm::radians(theta)) * cos(glm::radians(phi));
		position.y = r * sin(glm::radians(theta)) * sin(glm::radians(phi));
		position.z = r * cos(glm::radians(theta));
	}

	// function to get the view matrix
	glm::mat4 getViewMatrix() {
		// eye vector: from position of the camera to the target (in this case, origin)
		glm::vec3 eye = glm::normalize(target - position);
		// right vector: (0, 1, 0) cross eye vector
		glm::vec3 right = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), eye));
		// up vector: eye cross right
		glm::vec3 up = glm::cross(eye, right);
		// pass the above three to lookAt and this generates the view matrix
		return glm::lookAt(position, target, up);
	}

	glm::mat4 getProjectionMatrix(float aspectRatio) {
		return glm::perspective(glm::radians(fov), aspectRatio, 0.1f, 100.0f);
	}

	glm::mat4 getOrthoMatrix(float aspectRatio) {
		float orthoWidth = orthoRight - orthoLeft;
		float orthoHeight = orthoTop - orthoBottom;
		float halfW = orthoWidth / 2.0f;
		float halfH = orthoHeight / 2.0f;
		return glm::ortho(-halfW * aspectRatio, halfW * aspectRatio, -halfH, halfH, orthoNear, orthoFar);
	}

	void processOrthoZoom(float zoomAmount) {
		orthoLeft += zoomAmount;
		orthoRight -= zoomAmount;
		orthoBottom += zoomAmount;
		orthoTop -= zoomAmount;
	}

	// scroll
	void processScroll(float yOffset) {
		//fov -= yOffset;
		//if (fov < 1.0f) fov = 1.0f;
		//if (fov > 90.0f) fov = 90.0f;

		float zoomSpeed = 0.1f;
		float zoomAmount = yOffset * zoomSpeed;

		processOrthoZoom(zoomAmount);

	}

	// move 
	void moveCamera(glm::vec3 direction) {
		position -= direction;
		target -= direction;
	}
};

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
	//root->divided = false;
	//root->merged = false;
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

void splitBranch(SceneNode* root, CPU_Geometry branchGeometry, std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs, std::vector<std::pair<SceneNode*, SceneNode*>> newPairs, int index, std::vector<SceneNode*>& branchingStructure, bool addContour) {
	pairs.clear();
	root->labelBranches(root, pairs, index);
	newPairs.clear();
	index = 0;
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
	root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
	if (addContour) {
		bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
		branchingStructure.clear();
		accumulateBranchingStructure(root, branchingStructure);
	}
}

// NOT USED ANYMORE
void insertNode(SceneNode* node, CPU_Geometry branchGeometry, std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs, std::vector<std::pair<SceneNode*, SceneNode*>> newPairs, int index, std::vector<SceneNode*>& branchingStructure, bool addContour, int subdivisionCounter) {
	if (!node) return;

	std::vector<SceneNode*> originalChildren = node->children;

	for (SceneNode* child : originalChildren) {
		if (node->trackOriginalBranch && child->trackOriginalBranch) {
			SceneNode* splitterNode = new SceneNode();
			splitterNode->trackOriginalBranch = true;
			node->divideBranch(node, 0.1f, 2, subdivisionCounter);
			subdivisionCounter++;
			//splitBranch(node, branchGeometry, bindings, pairs, newPairs, index, branchingStructure, addContour);
			//std::cout << "node inserted" << std::endl;
			//node->removeChild(child);
			//node->addChild(splitterNode);
			//splitterNode->addChild(child);
			//splitterNode->localTranslation = glm::mat4(1.0f);
			//splitterNode->localRotation = glm::mat4(1.0f);
			//splitterNode->localScaling = glm::mat4(1.0f);
		}

		insertNode(child, branchGeometry, bindings, pairs, newPairs, index, branchingStructure, addContour, subdivisionCounter);
	}
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

		std::cout << "World position: (" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")\n";
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

int main() {
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

	CPU_Geometry branchGeometry;
	std::vector<CPU_Geometry> branchUpdates;
	std::vector<SceneNode*> branchingStructure;

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float>> edgeTransformations = SceneNode::extractEdgeTransforms("D:/Code/C++/NewPhytologist2017/plyFile/transform_matrices6.txt");
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
	root->getBranches(root, newPairs);
	// catmullrom gives smooth curve, linear gives sharp curve
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourCatmullRomGrouped(contour, 25, pairs);
	//std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourLinearGrouped(contour, 5, pairs);
	//std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> groupedContour = root->contourQuadraticGrouped(contour, 1, pairs);
	std::vector<ContourBinding> bindings = root->bindInterpolatedContourToBranches(groupedContour);
	//std::vector<glm::vec3> contours;
	//for (int i = 0; i < groupedContour.size(); i++) {
	//	for (int j = 0; j < groupedContour[i].first.size(); j++) {
	//		contours.push_back(groupedContour[i].first[j]);
	//	}
	//}
	//std::vector<ContourBinding> bindings = root->bindContourToBranches(contours, root, newPairs);
	// DEBUGGING PURPOSES
	CPU_Geometry mappingLines;

	float lastTime = glfwGetTime();
	bool sPressed = false;
	bool aKeyPressedLastFrame = false;
	bool tKeyPressedLastFrame = false;
	int branchCounter = 0;
	bool screenshotRequested = false;
	bool bidirectionalGrowth = false;

	while (!glfwWindowShouldClose(window)) {
		bool g_pressed = false;
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
		int state = glfwGetKey(window, GLFW_KEY_G);
		if (state == GLFW_PRESS)
		{
			g_pressed = true;
			root->animate(deltaTime);
		}

		// split branch
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)  //  && !sPressed to only execute once per press 
		{
			sPressed = true;
			if (root->divideBranch(root, .01f, 2.f, bidirectionalGrowth = false)) {
				splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, true);
				//pairs.clear();
				//root->labelBranches(root, pairs, index);
				//newPairs.clear();
				//index = 0;
				//root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
				//root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
				//bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
				//branchingStructure.clear();
				//accumulateBranchingStructure(root, branchingStructure);
				//root->printStructure(root);
				//printf("--------------------\n");

				//for (int i = 0; i < bindings.size(); i++) {
				//	printMat4(bindings[i].childNode->globalTransformation);
				//}
				//printf("--------------------\n");

				resetBool(root);
			}
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_RELEASE) {
			sPressed = false;
		}

		// merge branch
		if (clickedToRemove) {
			for (int i = 0; i < branchingStructure.size(); i++) {
				// clicked on a point
				if ((abs(worldPos.x - branchingStructure[i]->globalTransformation[3].x) <= 0.05) && (abs(worldPos.y - branchingStructure[i]->globalTransformation[3].y) <= 0.05)) {
					// clicked point is a branching point 
					if (branchingStructure[i]->parent != NULL && !branchingStructure[i]->children.empty()) {
						glm::vec3 parentBranch = glm::vec3(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]);
						for (int j = 0; j < branchingStructure[i]->children.size(); j++) {
							glm::vec3 childBranch = glm::vec3(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3]);
							float dotProduct = glm::dot(parentBranch, childBranch);
							// find the correct parent and child branch node to merge
							if (dotProduct == glm::length(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]) * glm::length(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3]) ||
								dotProduct == -glm::length(branchingStructure[i]->globalTransformation[3] - branchingStructure[i]->parent->globalTransformation[3]) * glm::length(branchingStructure[i]->children[j]->globalTransformation[3] - branchingStructure[i]->globalTransformation[3])) {
								root->mergeBranch(root, branchingStructure[i], branchingStructure[i]->parent, branchingStructure[i]->children[j]);
								root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
								root->rebindContourWithMergedBranch(root, bindings);
								resetBool(root);
							}
							break;
						}
					}
				}
			}
			clickedToRemove = false;
		}

		// add branch based on clicking
		if (clickedToAdd) {
			for (int i = 0; i < bindings.size(); i++) {
				// clicked on a point
				if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-02) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-02)) {
					ContourBinding* c = &bindings[i];
					// don't want to add a new branch relative to the contour point that is binded to leaf node (don't want a vertical branch) -> might not need?
					if (!(c->childNode->children.empty())) {
						root->divideBranchMinDistance(root, c);
						splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);

						// add new branch
						SceneNode* newNode = root->addNewBranch(root, c);
						pairs.clear();
						root->labelBranches(root, pairs, index);
						newPairs.clear();
						index = 0;
						root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
						root->rebindToNewBranch(newNode, c, bindings, 0.1f);
						//insertNode(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);

						resetBool(root);
					}
					break;
				}
			}
			clickedToAdd = false;
		}

		// add branch based on position
		int currentCKeyState = glfwGetKey(window, GLFW_KEY_A); // only execute once per frame
		if (currentCKeyState == GLFW_PRESS && !aKeyPressedLastFrame)
		{
			float vs[] = { 0.994907, 1.237082 };
			std::random_device rd;
			std::mt19937 gen(rd()); // random number
			float mainAxisLength = computeMainAxisLength(root);
			std::uniform_real_distribution<float> dist(0.3f, computeMainAxisLength(root));
			//ContourBinding* c = root->findContourPointToAddBranch(dist(gen), root, bindings);

			//ContourBinding* c = root->findContourPointToAddBranch(vs[branchCounter], root, bindings);
			branchCounter = (branchCounter + 1) % 2;

			ContourBinding* c = &bindings[98];
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
					splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);
					//pairs.clear();
					//root->labelBranches(root, pairs, index);
					//newPairs.clear();
					//index = 0;
					//root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					//root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);

					// add new branch
					SceneNode* newNode = root->addNewBranch(root, c);
					pairs.clear();
					root->labelBranches(root, pairs, index);
					newPairs.clear();
					index = 0;
					// need to update branch
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					root->rebindToNewBranch(newNode, c, bindings, 0.1f);
					//insertNode(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);

					resetBool(root);
				}
				else {
					printf("no branch added\n");
					root = originalRoot;
					bindings = root->rebindContour(originalBindings, nodeMap);
					resetBool(root);
				}
			}
		}
		aKeyPressedLastFrame = (currentCKeyState == GLFW_PRESS);

		// subdivide and add branch all at once
		int tKeyState = glfwGetKey(window, GLFW_KEY_T);
		if (tKeyState == GLFW_PRESS && !tKeyPressedLastFrame) {
			for (int i = 0; i < 10; i++) {
				if (root->divideBranch(root, .1f, 2.f, bidirectionalGrowth)) {
					splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, true);
					resetBool(root);
				}
			}
			float vs[] = { 0.994907, 1.237082 };
			std::random_device rd;
			std::mt19937 gen(rd()); // random number
			float mainAxisLength = computeMainAxisLength(root);
			std::uniform_real_distribution<float> dist(0.3f, computeMainAxisLength(root));
			//ContourBinding* c = root->findContourPointToAddBranch(dist(gen), root, bindings);

			//ContourBinding* c = root->findContourPointToAddBranch(vs[branchCounter], root, bindings);
			branchCounter = (branchCounter + 1) % 2;

			ContourBinding* c = &bindings[98];
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
					splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);

					// add new branch
					SceneNode* newNode = root->addNewBranch(root, c);
					pairs.clear();
					root->labelBranches(root, pairs, index);
					newPairs.clear();
					index = 0;
					// need to update branch
					root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
					//root->printStructure(root);;
					root->rebindToNewBranch(newNode, c, bindings, 0.1f);
					//root->printStructure(root);
					//insertNode(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);

					resetBool(root);
				}
				else {
					printf("no branch added\n");
					root = originalRoot;
					bindings = root->rebindContour(originalBindings, nodeMap);
					resetBool(root);
				}
			}
		}
		tKeyPressedLastFrame = (tKeyState == GLFW_PRESS);

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
		//root->interpolateBranchTransforms(p, branchUpdates);

		// add contour point and bind
		branchingStructure.clear();
		accumulateBranchingStructure(root, branchingStructure);
		bindings = root->addContourPoints(bindings);
		//bindings = root->snapContourPoints(bindings);
		if (g_pressed) {
			root->animationPerFrame(bindings, deltaTime);
			root->calculateNormalDirection(bindings);  // need to update normal direction whenever there is animation
		}

		mappingLines.verts.clear();
		mappingLines.indices.clear();
		for (int i = 0; i < bindings.size(); i++) {
			int startIdx = mappingLines.verts.size();
			mappingLines.verts.push_back(bindings[i].contourPoint - glm::vec3(0, 0, 0.02));
			mappingLines.verts.push_back(bindings[i].closestPoint - glm::vec3(0, 0, 0.02));
			mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));
			mappingLines.cols.push_back(glm::vec3(0.8f, 0.6f, 0.8f));
			mappingLines.indices.push_back(startIdx);     // from contour
			mappingLines.indices.push_back(startIdx + 1); // to closest branch point
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
		//	glDrawArrays(GL_POINTS, 0, branchUpdates[i].verts.size());
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
		// saving contour and branch information
		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
			////taking screenshot
			//int width, height;
			//glfwGetFramebufferSize(window, &width, &height);
			//saveScreenshot(width, height);
			//screenshotRequested = false;

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

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

