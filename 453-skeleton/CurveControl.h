#pragma once

#include <memory>

#include "Geometry.h"
#include "Panel.h"
#include "ShaderProgram.h"
#include "Window.h"

// forward declare classes that only exist in the cpp file
class CurveEditorCallBack;
class TurnTable3DViewerCallBack;

class CurveEditorPanelRenderer;

enum class ViewOption { CurveEditor, SurfaceOfRevolution, TensorSurface };

class CurveControl {
public:
	explicit CurveControl(Window& window);

<<<<<<< Updated upstream
	void Update();
	void DrawGeometry();
=======
	bool g_pressed = false;

	void init(const std::string& path, bool isTxt);
	void step(float dt);

	void stepHeadless(float dt, float length);

	void updateSimulation(float dt);
	void animateRebuild(float dt);
	void rebuildContourGeometry();
	void rebuildDebugGeometry();

	void handleSKey();
	void handleGKey(float dt);
	void handleRemoveBranchClick(const glm::vec3& worldPos, bool& mouseClicked);
	void handleAddBranchClick(const glm::vec3& worldPos, bool& mouseClicked);
	void simulateGrowth(float dt);
	void simulateSubdivision(float length, float dt);

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
>>>>>>> Stashed changes

private:
	// State variables
	ViewOption mCurrentViewOption = ViewOption::CurveEditor;
	std::shared_ptr<CurveEditorCallBack> mCurveControls;
	std::shared_ptr<TurnTable3DViewerCallBack> m3DCameraControls;
	ShaderProgram mShader;
	Panel mPanel;
	std::shared_ptr<CurveEditorPanelRenderer> mPanelRenderer;

<<<<<<< Updated upstream
	// Geometry
	CPU_Geometry mCurveGeometry;
	GPU_Geometry mGPUGeometry;
	GPU_Geometry mPointGPUGeometry;

	// Private functions
	CPU_Geometry GenerateInitialGeometry();
=======
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

>>>>>>> Stashed changes
};
