#pragma once

#include <memory>

#include "Geometry.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// define the class before the struct since it uses the class
class SceneNode;

struct ContourBinding {
	SceneNode* parentNode;   
	SceneNode* childNode;
	glm::vec3 contourPoint;
	float t;                 
	glm::vec3 closestPoint;
	glm::mat4 previousAnimateInverse;
	std::vector<float> weights;
	std::vector<glm::mat4> previousAnimateInverseMat;
	int childBranchIndex;
};

struct TransformData {
	glm::mat4 rotation;
	glm::mat4 scaling;
	glm::mat4 translation;
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
	std::vector<ContourBinding> bindContourToMultipleBranches(const std::vector<glm::vec3>& contourPoints, SceneNode* root, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments);
	std::vector<glm::vec3> animateContour(std::vector<ContourBinding>& bindings);
	static void getLeafNodes(SceneNode* node, std::vector<SceneNode*>& leaves);
	static std::vector<glm::vec3> generateInitialContourControlPoints(SceneNode* root);
	static std::vector<glm::vec3> bSplineCurve(int iterations, SceneNode* root);
	static std::vector<glm::vec3> contourCatmullRom(std::vector<glm::vec3> root, int points);
	void interpolateBranchTransforms(std::vector<std::pair<SceneNode*, SceneNode*>>& pair, std::vector<CPU_Geometry>& outGeometry);
	std::vector<glm::vec3> distanceBetweenContourPoints(std::vector<glm::vec3> contourPoints);
	void inverseTransform(std::vector<ContourBinding>& bindings);
	std::vector<ContourBinding> addContourPoints(std::vector<ContourBinding>& bindings);
	void animationPerFrame(std::vector<ContourBinding>& bindings);
	void multipleWeights(std::vector<ContourBinding>& bindings);
	void animationPerFrameUsingMultipleWeights(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments);
	void bindToBranchingPoint(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments);
	void handleMouseClick(double xpos, double ypos, int screenWidth, int screenHeight,
		glm::mat4 view, glm::mat4 projection, std::vector<glm::vec3> contourPoints, CPU_Geometry geom);

	void generateContourPoints(SceneNode* node, std::vector<glm::vec3>& controlPoints);
	static std::vector<std::vector<glm::vec3>> contourCatmullRomGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment);
	std::vector<ContourBinding> bindContourToBranches(const std::vector<std::vector<glm::vec3>>& contourPoints, SceneNode* root, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments);

	std::tuple<SceneNode*, SceneNode*> findChildrenOfFirstCommonAncestorFromRoot(
		SceneNode* root,
		const ContourBinding& a,
		const ContourBinding& b);
	std::vector<ContourBinding> bindInterpolatedContourToBranches(const std::vector<std::vector<glm::vec3>>& contourPoints, SceneNode* root, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments);
	std::vector<glm::vec3> midPoints(std::vector<glm::vec3>& contourPoints);

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
