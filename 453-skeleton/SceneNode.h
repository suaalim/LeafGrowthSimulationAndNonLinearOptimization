#pragma once

#include <memory>

#include "Geometry.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>


void printMat4(const glm::mat4& mat);
// define the class before the struct since it uses the class
class SceneNode;

struct CrossoverRegion
{
	int startIndex;
	int endIndex;
};

struct BranchKey
{
	SceneNode* parent;
	SceneNode* child;

	bool operator==(const BranchKey& other) const
	{
		return parent == other.parent && child == other.child;
	}
};

struct BranchKeyHash
{
	std::size_t operator()(const BranchKey& k) const
	{
		return std::hash<SceneNode*>()(k.parent) ^ (std::hash<SceneNode*>()(k.child) << 1);
	}
};

enum class TraversalType
{
	Down,
	Up
};

struct TraversalEvent
{
	BranchKey key;
	TraversalType type;
	int index;
};

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
	bool newBranchBinding = false;
	float blending;
	// compute normal direction
	glm::vec3 normalDirection = glm::vec3(0.f);
	float normalFactor = 0.05f;
};

struct Branch {
	SceneNode* root = nullptr;
	SceneNode* leaf = nullptr;
	std::vector<SceneNode*> nodes; // full path root -> leaf
};

// SceneNode for Scene Graph
class SceneNode {
public:
	SceneNode();
	static SceneNode* cloneSceneNode(SceneNode* node, SceneNode* parent, std::unordered_map<SceneNode*, SceneNode*>& nodeMap);
	std::vector<ContourBinding> rebindContour(const std::vector<ContourBinding>& bindings, const std::unordered_map<SceneNode*, SceneNode*>& nodeMap);
	void addChild(SceneNode* child);
	void removeChild(SceneNode* childToRemove);
	static SceneNode* createBranchingStructure(int depth, std::vector<std::vector<int>> parentChildPairs, std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> transformations);
	static std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> extractEdgeTransformsTxt(const std::string& filename);
	static std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> extractEdgeTransformsToml(const std::string& filename);
	static std::vector<std::vector<int>> buildChildrenList(
		const std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>>& edges
	);
	void updateBranch(const glm::mat4& parentTransform, const glm::mat4& parentTransformAnimation, const glm::mat4& parentRestInverse, const glm::mat4& parentRest, CPU_Geometry& outGeometry);
	void animate(float deltaTime);
	void deleteSceneGraph(SceneNode* node);
	void getBranches(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments);
	void labelBranches(SceneNode* node, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& segments, int& i);
	void printBranches(SceneNode* node);
	static void getLeafNodes(SceneNode* node, std::vector<SceneNode*>& leaves);
	static std::vector<glm::vec3> generateInitialContourControlPoints(SceneNode* root);
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> contourCatmullRomGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches);
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> contourLinearGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches);
	std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>> contourQuadraticGrouped(std::vector<glm::vec3> controlPoints, int pointsPerSegment, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& branches);
	std::vector<glm::vec3> midPoints(std::vector<glm::vec3>& contourPoints);
	std::vector<ContourBinding> bindInterpolatedContourToBranches(std::vector<std::pair<std::vector<glm::vec3>, std::pair<SceneNode*, SceneNode*>>>& contourPoints);
	void interpolateBranchTransforms(std::vector<std::pair<SceneNode*, SceneNode*>>& pair, std::vector<CPU_Geometry>& outGeometry);
	void animationPerFrame(std::vector<ContourBinding>& bindings, float deltaTime);

	bool divideBranch(SceneNode* node, float threshold, float division, bool bidirectionalGrowth);
	void rebindContourWithBrokenBranch(SceneNode* node, std::vector<std::pair<SceneNode*, SceneNode*>>& segments, int& i, std::vector<ContourBinding>& bindings);
	ContourBinding* findContourPointToAddBranch(float height, SceneNode* root, std::vector<ContourBinding>& contourPoints);
	SceneNode* addNewBranch(SceneNode* node, ContourBinding* contour, int& maxID);
	ContourBinding findBestBinding(SceneNode* root, const glm::vec3& contourPoint);
	bool splitAtBinding(ContourBinding* pointToBreak);
	void rebindToNewBranch(SceneNode* newNode, ContourBinding* contour, std::vector<ContourBinding>& bindings, float dist);
	glm::quat accumulateRotationToRoot(SceneNode* node);
	std::vector<ContourBinding> addNewContourToBindToNewBranchNode(std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs);
	std::vector<ContourBinding> snapContourPoints(std::vector<ContourBinding>& bindings);
	std::vector<ContourBinding> addContourPoints(std::vector<ContourBinding>& bindings);
	bool mergeBranch(SceneNode* node, SceneNode* nodeToRemove, SceneNode* parentToMerge, SceneNode* childToMerge);
	void rebindContourWithMergedBranch(SceneNode* node, std::vector<ContourBinding>& bindings);
	void calculateNormalDirection(std::vector<ContourBinding>& bindings);
	void printStructure(SceneNode* node);
	void printTree(SceneNode* node, int depth);
	void buildBranches(SceneNode* node, std::vector<SceneNode*>& currentPath, std::vector<Branch>& branches);
	std::vector<Branch> generateAllBranches(SceneNode* root);
	std::vector<ContourBinding> bindContourToBranches(const std::vector<glm::vec3>& contourPoints, SceneNode* root, std::vector<std::pair<SceneNode*, SceneNode*>>& segments);
	int getMaxID(SceneNode* node);
	void incrementBranchIDsOnAxis(SceneNode* node, int axisID);
	void decrementBranchIDsOnAxis(SceneNode* node, int axisID);
	void reorganizeChildrenLeft(SceneNode* node);
	void reorganizeChildrenRight(SceneNode* node);
	bool divideBranchMinDistance(SceneNode* node, ContourBinding* contour);
	void rebindContourPointsLargeBinding(std::vector<ContourBinding>& bindings, SceneNode* root);
	std::vector<ContourBinding> addContourPointsLargeBinding(std::vector<ContourBinding>& bindings);
	void validateBindingsDFS(
		SceneNode* node,
		std::vector<ContourBinding*>& bindings,
		int& contourPos,
		std::vector<std::pair<int, BranchKey>>& misorientedPoints);
	std::vector<std::pair<int, BranchKey>> findMisorientedContourIndices(
		SceneNode* root,
		std::vector<ContourBinding*>& bindings);

	//glm::mat4 globalTransformationBranch = glm::mat4(1.f);
	// global transformation for contour A = T*V
	glm::mat4 globalTransformation = glm::mat4(1.0f);
	glm::mat4 marginTransformation = glm::mat4(1.0f);
	// global to local transformation for rest pose 
	glm::mat4 restPoseInverse;
	// rest pose
	glm::mat4 restPose;
	// for rebinding
	bool midBranch = false;
	// for knowing where to add a new branch/distinguish new branch 
	bool addBranch = false;
	// for rebinding group of contour points
	bool trackOriginalBranch = false;
	// for rebinding after merging
	bool toMerge = false;
	bool divided = false;
	bool merged = false;

	SceneNode* parent;
	std::vector<SceneNode*> children;

	std::unordered_map<int, SceneNode*> nodes;
	std::unordered_map<int, std::vector<std::pair<int, TransformData>>> edges;
	int axisID = 0;
private:
	// T trasformation (rest pose)
	glm::mat4 localTranslation;
	glm::quat localRotation;
	glm::mat4 localScaling;
	// V transformation (animation)
	glm::mat4 animateTranslation = glm::mat4(1.0f);
	glm::quat animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::mat4 animateScaling = glm::mat4(1.0f);
	// expansion 
	glm::mat4 expansion = glm::mat4(1.f);
	// growth
	glm::mat4 growth = glm::mat4(1.f);
	// animation variables
	float deltatime = 0.0f;
	float animationDirection = 1.0f; // left and right branch rotation (+angle, -angle)
	float animationAngle = 0.0f;
	float animationTime = 0.0f;
	float animationDuration = 2.5f; // how long the animation lasts
	float rotationAngle = 0.f;
	float animationScaling = 1.0f;
	float S = 1.f;
	float expansionAmount = 1.0f;
	float expansionFactor = 1.f;
	float growthAmount = 1.0f;
	float growthFactor = 1.0f;
	float positionOnBranch = 1.f; // to calculate positional information

	int branchID = 0;
	float distanceFromRoot;
};

