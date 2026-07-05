#pragma once

#include <vector>
#include <tuple>

#include "SceneNode.h"
#include "Geometry.h"

#ifdef HEADLESS_BUILD
#define GLFW_CALL(x)   // no-op
#else
#define GLFW_CALL(x) x
#endif

class Simulation {
public:
	CPU_Geometry branchGeometry;
	CPU_Geometry contourGeometry;
	CPU_Geometry mappingLines;

	bool g_pressed = false;

	float init(const std::string& path, bool isTxt, const std::string& newBranch, const std::string& sim);

	void stepHeadless(float dt, float length);

	void updateSimulation();
	void animateRebuild(float dt);
	void rebuildContourGeometry();
	void rebuildDebugGeometry();
	void rebuildBranchGeometry();
	void clearGeometry();

	void handleSKey();
	void pressSKey(int state);
	void releaseSkey(int state);
	void handleGKey(float dt);
	void pressGKey(float dt, int state);
	void releaseGKey(int state);
	void handleRemoveBranchClick(const glm::vec3& worldPos, bool mouseClicked);
	void handleAddBranchClick(const glm::vec3& worldPos, bool mouseClicked, bool perpendicular);
	void simulateGrowth(float dt);
	void simulateSubdivision();
	void screenshot(GLFWwindow* window);
	void saveContourGeometry(GLFWwindow* window);
	void handleAKey(float dt);
	void simulationInstructions(float dt);

	void splitBranch(
		SceneNode* root,
		CPU_Geometry& branchGeometry,
		std::vector<ContourBinding>& bindings,
		std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs,
		std::vector<std::pair<SceneNode*, SceneNode*>>& newPairs,
		int index,
		std::vector<SceneNode*>& branchingStructure,
		bool addContour
	);
	float computeMainAxisLength(SceneNode* root);
	void resetBool(SceneNode* root);
	void accumulateBranchingStructure(
		SceneNode* root,
		std::vector<SceneNode*>& branchingStructure
	);

private:
	SceneNode* root;

	std::vector<ContourBinding> bindings;
	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> edgeTransforms;
	std::vector<CPU_Geometry> branchUpdates;

	//CPU_Geometry branchGeometry;
	//CPU_Geometry contourGeometry;
	//CPU_Geometry mappingLines;

	std::vector<std::pair<SceneNode*, SceneNode*>> branchPairs;

	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	std::vector<SceneNode*> branchingStructure;

	bool sPressed = false;
	float deltaTime = 0.0f;
	int index = 0;
	bool subdivisionDone = false;
	int maxID = 0;
	bool screenshotRequested = false;
};
