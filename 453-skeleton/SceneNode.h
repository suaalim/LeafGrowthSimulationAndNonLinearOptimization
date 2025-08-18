#pragma once

#include <memory>

#include "Geometry.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>

// define the class before the struct since it uses the class
class SceneNode;

struct TransformData {
	glm::mat4 rotation;
	glm::mat4 scaling;
	glm::mat4 translation;
};

struct ContourBinding {
	SceneNode* parentNode;
	SceneNode* childNode;
	glm::vec3 contourPoint;
	float t;
	glm::vec3 closestPoint;
	glm::mat4 previousAnimateInverse;
	int childBranchIndex;
};

// SceneNode for Scene Graph
class SceneNode {
public:
	SceneNode();
	void addChild(SceneNode* child);
	static SceneNode* createBranchingStructure(int depth, std::vector<std::vector<int>> parentChildPairs, std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float>> transformations);
	static std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float>> extractEdgeTransforms(const std::string& filename);
	static std::vector<std::vector<int>> buildChildrenList(
		const std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float>>& edges
	);
	void updateBranch(const glm::mat4& parentTransform, const glm::mat4& parentRestInverse, const glm::mat4& parentRest, CPU_Geometry& outGeometry);
	void animate(float deltaTime);
	void deleteSceneGraph(SceneNode* node);
	static glm::vec3 intersectionPoint(glm::vec3 P, glm::vec3 Q, glm::vec3 R);
	void getBranches(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments);
	void labelBranches(SceneNode* node, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments, int& i);
	static void getLeafNodes(SceneNode* node, std::vector<SceneNode*>& leaves);
	static std::vector<glm::vec3> generateInitialContourControlPoints(SceneNode* root);
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> contourCatmullRomGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches);
	std::vector<glm::vec3> midPoints(std::vector<glm::vec3>& contourPoints);
	std::vector<ContourBinding> bindInterpolatedContourToBranches(std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>>& contourPoints);
	std::tuple<SceneNode*, SceneNode*> findChildrenOfFirstCommonAncestorFromRoot(
		SceneNode* root,
		const ContourBinding& a,
		const ContourBinding& b
	);
	void interpolateBranchTransforms(std::vector<std::pair<SceneNode*, SceneNode*>>& pair, std::vector<CPU_Geometry>& outGeometry);
	std::vector<glm::vec3> distanceBetweenContourPoints(std::vector<glm::vec3> contourPoints);
	std::vector<ContourBinding> addContourPoints(std::vector<ContourBinding>& bindings);
	void animationPerFrame(std::vector<ContourBinding>& bindings);
	void handleMouseClick(double xpos, double ypos, int screenWidth, int screenHeight,
		glm::mat4 view, glm::mat4 projection, std::vector<glm::vec3> contourPoints, CPU_Geometry geom);

	// global transformation A = T*V
	glm::mat4 globalTransformation = glm::mat4(1.0f);
	// global to local transformation for rest pose 
	glm::mat4 restPoseInverse;
	// rest pose
	glm::mat4 restPose;
	// has contour been updated?
	bool contourChanged = false;

	SceneNode* parent;
	std::vector<SceneNode*> children;

	std::unordered_map<int, SceneNode*> nodes;
	std::unordered_map<int, std::vector<std::pair<int, TransformData>>> edges;

private:
	// T trasformation (rest pose)
	glm::mat4 localTranslation;
	glm::quat localRotation;
	glm::mat4 localScaling;
	// V transformation (animation)
	glm::mat4 animateTranslation = glm::mat4(1.0f);
	glm::quat animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::mat4 animateScaling = glm::mat4(1.0f);
	// animation variables
	float deltatime = 0.0f;
	float animationDirection = 1.0f; // control how left and right branches move differently (+angle, -angle)
	float animationAngle = 0.0f;
	float animationScaling = 1.0f;
	float animationTime = 0.0f;
	float animationDuration = 0.5f; // how long the animation lasts
	float S = 1.f;
	float rotationAngle = 0.f;

};
