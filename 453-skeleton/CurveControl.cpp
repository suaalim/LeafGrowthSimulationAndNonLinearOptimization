#include "CurveControl.h"
#include "SceneNode.h"
#include <vector>
#include <glm/glm.hpp>
#include <utility>      // std::pair
#include <tuple>        // std::tuple
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <glm/gtx/string_cast.hpp>

class CurveEditorCallBack : public CallbackInterface {
public:
	CurveEditorCallBack() {}

	virtual void keyCallback(int key, int scancode, int action, int mods) override {
		Log::info("KeyCallback: key={}, action={}", key, action);
	}

	virtual void mouseButtonCallback(int button, int action, int mods) override {
		Log::info("MouseButtonCallback: button={}, action={}", button, action);

		if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
			wasClicked = true;
		}
	}

void Simulation::splitBranch(SceneNode* root, CPU_Geometry& branchGeometry, std::vector<ContourBinding>& bindings, std::vector<std::tuple<SceneNode*, SceneNode*, int>>& pairs, std::vector<std::pair<SceneNode*, SceneNode*>>& newPairs, int index, std::vector<SceneNode*>& branchingStructure, bool addContour)
{
	pairs.clear();
	root->labelBranches(root, pairs, index);
	newPairs.clear();
	index = 0;
	root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
	root->rebindContourWithBrokenBranch(root, newPairs, index, bindings);
	//if (addContour) {
	//	bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
	//	branchingStructure.clear();
	//	accumulateBranchingStructure(root, branchingStructure);
	//}
}

	virtual void scrollCallback(double xoffset, double yoffset) override {
		Log::info("ScrollCallback: xoffset={}, yoffset={}", xoffset, yoffset);
	}

	virtual void windowSizeCallback(int width, int height) override {
		Log::info("WindowSizeCallback: width={}, height={}", width, height);

	root = SceneNode::createBranchingStructure(
		0, parentChildPairs, edgeTransforms
	);

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		branchGeometry
	);

	auto contour = root->generateInitialContourControlPoints(root);  // generate contour points associated to root and leaf nodes
	contour = root->midPoints(contour);
	
	std::vector<std::tuple<SceneNode*, SceneNode*, int>> pairs;
	int idx = 0;
	root->labelBranches(root, pairs, idx);

	std::vector<std::pair<SceneNode*, SceneNode*>> newPairs;
	root->getBranches(root, newPairs);

	auto grouped = root->contourCatmullRomGrouped(contour, 25, pairs);

	bindings = root->bindInterpolatedContourToBranches(grouped);
}

	bool wasClicked = false;
};

// Can swap the callback instead of maintaining a state machine

class TurnTable3DViewerCallBack : public CallbackInterface {
public:
	TurnTable3DViewerCallBack() {}

	virtual void keyCallback(int key, int scancode, int action, int mods) {}
	virtual void mouseButtonCallback(int button, int action, int mods) {}
	virtual void cursorPosCallback(double xpos, double ypos) {}
	virtual void scrollCallback(double xoffset, double yoffset) {}
	virtual void windowSizeCallback(int width, int height) {

		// The CallbackInterface::windowSizeCallback will call glViewport for us
		CallbackInterface::windowSizeCallback(width, height);
	}
};

class CurveEditorPanelRenderer : public PanelRendererInterface {
public:
	CurveEditorPanelRenderer()
		: inputText(""), buttonClickCount(0), pointSize(5.0f), dragValue(0.0f),
		inputValue(0.0f), checkboxValue(false), comboSelection(0) {
		// Initialize options for the combo box
		options[0] = "Option 1";
		options[1] = "Option 2";
		options[2] = "Option 3";

		// Initialize color (white by default)
		colorValue[0] = 1.0f; // R
		colorValue[1] = 1.0f; // G
		colorValue[2] = 1.0f; // B
	}

	root->updateBranch(
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		glm::mat4(1.0f),
		branchGeometry
	);
	root->updateGrowthRateForMidNode(root);

		// Text input
		ImGui::InputText("Input Text", inputText, IM_ARRAYSIZE(inputText));

		// Display the input text
		ImGui::Text("You entered: %s", inputText);

		// Button
		if (ImGui::Button("Click Me")) {
			buttonClickCount++;
		}
		ImGui::Text("Button clicked %d times", buttonClickCount);

		// Scrollable block
		ImGui::TextWrapped("Scrollable Block:");
		ImGui::BeginChild("ScrollableChild", ImVec2(0, 100),
			true); // Create a scrollable child
		for (int i = 0; i < 20; i++) {
			ImGui::Text("Item %d", i);
		}
		ImGui::EndChild();

		// Float slider
		ImGui::SliderFloat("Float Slider", &pointSize, 5.0f, 100.0f,
			"Point Size: %.3f");

		// Float drag
		ImGui::DragFloat("Float Drag", &dragValue, 0.1f, 0.0f, 100.0f,
			"Drag Value: %.3f");

		// Float input
		ImGui::InputFloat("Float Input", &inputValue, 0.1f, 1.0f,
			"Input Value: %.3f");

		// Checkbox
		ImGui::Checkbox("Enable Feature", &checkboxValue);
		ImGui::Text("Feature Enabled: %s", checkboxValue ? "Yes" : "No");

		// Combo box
		ImGui::Combo("Select an Option", &comboSelection, options,
			IM_ARRAYSIZE(options));
		ImGui::Text("Selected: %s", options[comboSelection]);

		// Displaying current values
		ImGui::Text("Point Size: %.3f", pointSize);
		ImGui::Text("Drag Value: %.3f", dragValue);
		ImGui::Text("Input Value: %.3f", inputValue);
	}
}

void Simulation::simulateGrowth(float dt)
{;
	g_pressed = true;
	root->animate(dt);
}


void Simulation::simulateSubdivision(float length, float dt)
{
	while (root->divideBranch(root, 0.025f, 2.f, false)) {
		splitBranch(
			root,
			branchGeometry,
			bindings,
			pairs,
			newPairs,
			index = 0,
			branchingStructure,
			true
		);
		resetBool(root);
		simulateGrowth(dt);
		updateSimulation(dt);
		//if (g_pressed) animateRebuild(dt);
	}
}

//void Simulation::handleSKey()
//{
//	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);
//
//	if (state == GLFW_PRESS) {
//
//		if (root->divideBranch(root, 0.01f, 2.f, false)) {
//
//			splitBranch(
//				root,
//				branchGeometry,
//				bindings,
//				pairs,
//				newPairs,
//				index = 0,
//				branchingStructure,
//				true
//			);
//
//			resetBool(root);
//		}
//	}
//
//	if (state == GLFW_RELEASE) {
//		sPressed = false;
//	}
//}

void Simulation::handleSKey()
{
	int state = glfwGetKey(glfwGetCurrentContext(), GLFW_KEY_S);

private:
	float colorValue[3];    // Array for RGB color values
	char inputText[256];    // Buffer for input text
	int buttonClickCount;   // Count button clicks
	float pointSize;        // Value for float slider
	float dragValue;        // Value for drag input
	float inputValue;       // Value for float input
	bool checkboxValue;     // Value for checkbox
	int comboSelection;     // Index of selected option in combo box
	const char* options[3]; // Options for the combo box
};

CurveControl::CurveControl(Window& window)
	: mShader("shaders/test.vert", "shaders/test.frag"),
	mPanel(window.getGLFWwindow()) {
	mCurveControls = std::make_shared<CurveEditorCallBack>();
	m3DCameraControls = std::make_shared<TurnTable3DViewerCallBack>();

			if (root->divideBranch(root, 0.01f, 2.f, false)) {
				splitBranch(
					root,
					branchGeometry,
					bindings,
					pairs,
					newPairs,
					index = 0,
					branchingStructure,
					true
				);
				resetBool(root);
			}
		}
	}

	mCurveGeometry = GenerateInitialGeometry();
	mGPUGeometry.setVerts(mCurveGeometry.verts);
	mGPUGeometry.setCols(mCurveGeometry.cols);

	// Using two different buffers for the control points and the lines themselves
	// makes it easier to highlight the selected point
	mPointGPUGeometry.setVerts(mCurveGeometry.verts);
	mPointGPUGeometry.setCols(
		std::vector<glm::vec3>(mCurveGeometry.verts.size(), { 1.f, 0.f, 0.f }));

	if (state == GLFW_PRESS)
	{
		g_pressed = true;
		//root->updateGrowthRateForMidNode(root);
		root->animate(dt);
	}
}

void Simulation::handleRemoveBranchClick(const glm::vec3& worldPos, bool& mouseClicked)
{
	if (!mouseClicked) return; // or remove this flag entirely if moved outside

	for (int i = 0; i < branchingStructure.size(); i++)
	{
		SceneNode* node = branchingStructure[i];

		glm::vec3 nodePos = glm::vec3(node->globalTransformation[3]);

		// click proximity check
		if (glm::abs(worldPos.x - nodePos.x) <= 0.05f &&
			glm::abs(worldPos.y - nodePos.y) <= 0.05f)
		{
			if (node->parent != nullptr && !node->children.empty())
			{
				glm::vec3 parentBranch =
					glm::vec3(node->globalTransformation[3] -
						node->parent->globalTransformation[3]);

				for (SceneNode* child : node->children)
				{
					glm::vec3 childBranch =
						glm::vec3(child->globalTransformation[3] -
							node->globalTransformation[3]);

					float dotProduct = glm::dot(parentBranch, childBranch);

	glClearColor(backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	mShader.use();

	// Render the control points
	glPointSize(mPanelRenderer->getPointSize());
	mGPUGeometry.bind();
	glDrawArrays(GL_POINTS, 0, mCurveGeometry.verts.size());

	// Render the curve that connects the control points
	mPointGPUGeometry.bind();
	glDrawArrays(GL_LINE_STRIP, 0, mCurveGeometry.verts.size());

	// disable sRGB for things like imgui
	glDisable(GL_FRAMEBUFFER_SRGB);
	mPanel.render();
}

void CurveControl::Update() {
	// Use this function to process logic and update things based on user inputs
	//Example: generate a new control point

	if (mCurveControls->wasClicked) {
		Log::debug("Insert or select a control point based on the position clicked");
		mCurveControls->wasClicked = false;
	}
}

void Simulation::handleAddBranchClick(const glm::vec3& worldPos, bool& mouseClicked)
{
	if (mouseClicked) {
		ContourBinding* c = nullptr;
		for (int i = 0; i < bindings.size(); i++) {
			// clicked on a point
			if ((abs(worldPos.x - bindings[i].contourPoint.x) <= 1e-02) && (abs(worldPos.y - bindings[i].contourPoint.y) <= 1e-02)) {
				c = &bindings[i];
				break;
			}
		}
		if (c == nullptr) {
			std::cout << "contour point not clicked" << std::endl;
		}
		// don't want to add a new branch relative to the contour point that is bound to leaf node (don't want a vertical branch) -> might not need?
		else if (!(c->childNode->children.empty())) {  
			if (root->divideBranchMinDistance(root, c))
				splitBranch(root, branchGeometry, bindings, pairs, newPairs, index = 0, branchingStructure, false);
			// add new branch
			SceneNode* newNode = root->addNewBranch(root, c);
			pairs.clear();
			root->labelBranches(root, pairs, index);
			newPairs.clear();
			index = 0;
			root->updateBranch(glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), branchGeometry);
			root->rebindToNewBranch(newNode, c, bindings, 0.1f);
			//// add new contour point to split node (dont do this anymore)
			//bindings = root->addNewContourToBindToNewBranchNode(bindings, pairs);
			//branchingStructure.clear();
			//accumulateBranchingStructure(root, branchingStructure);
			// reorganize children (leftmost to rightmost)
			root->reorganizeChildren(root);
			//pairs.clear();
			//root->labelBranches(root, pairs, index);
			/*int index = 0;
			std::vector<TraversalEvent> traversalPath;
			root->buildBranchOrderDFS(root, traversalPath, index);
			std::vector<std::pair<int, int>> indices = root->findMisorientedContourIndices(bindings, traversalPath);
			std::cout << "incorrect indices: " << indices.size() << std::endl;
			std::cout << "points: " << bindings.size() << std::endl;*/

			//std::vector<std::pair<int, BranchKey>> mismatch = root->findMisorientedContourIndices(root, bindings);
			//std::cout << "mismatch indices size: " << mismatch.size() << std::endl;
			//std::cout << "points: " << bindings.size() << std::endl;


			resetBool(root);
		}
		mouseClicked = false;
	}
}

void Simulation::stepHeadless(float dt, float length)
{
	if (!subdivisionDone) {
		simulateSubdivision(length, dt);
		subdivisionDone = true;
	}
	simulateGrowth(dt);
	updateSimulation(dt);
	if (g_pressed) animateRebuild(dt);
	rebuildContourGeometry();
	rebuildDebugGeometry();
}
