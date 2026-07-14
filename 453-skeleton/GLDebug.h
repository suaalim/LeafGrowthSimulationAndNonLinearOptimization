#pragma once
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <tuple>

// singleton
class NewBranchConfig
{
public:
	static NewBranchConfig& instance()
	{
		static NewBranchConfig inst;
		return inst;
	}

	bool load(const std::string& filename);

	float getNewBranchS() const { return newBranchS; }
	float getNewBranchExpansionFactor() const { return newBranchExpansionFactor; }
	float getNewBranchGrowthFactor() const { return newBranchGrowthFactor; }

private:
	NewBranchConfig() = default;

	float newBranchS = 0.0f;
	float newBranchExpansionFactor = 0.0f;
	float newBranchGrowthFactor = 0.0f;
};

// singleton
class SimulationConfig
{
public:
	static SimulationConfig& instance()
	{
		static SimulationConfig inst;
		return inst;
	}

	bool load(const std::string& filename);

	float getSubdivisionThreshold() const { return subdivisionThreshold; }
	float getSubdivisionDivision() const { return subdivisionDivision; }
	float getNewContourPointThreshold() const { return newContourPointThreshold;  }
	float getNewBindingPointThreshold() const { return newBindingPointThreshold; }
	float getRebindContourDistance() const { return rebindContourDistance; }
	float getDeltaTime() const { return deltaTime; }
	int getPointsPerSegment() const { return pointsPerSegment; }
	int getBestBindingStep() const { return bestBindingStep; }
	int getsubdivideBranch() const { return subdivideBranch; }
	int getRebindEveryFrame() const { return rebindEveryFrame; }
	int getPerpendicularBranch() const { return perpendicularBranch; }

private:
	SimulationConfig() = default;

	float subdivisionThreshold = 0.0f;
	float subdivisionDivision = 0.0f;
	float newContourPointThreshold = 0.0f;
	float newBindingPointThreshold = 0.0f;
	float rebindContourDistance = 0.0f;
	float deltaTime = 0.0f;
	int pointsPerSegment = 0;
	int bestBindingStep = 0;
	int subdivideBranch = 0; // 0 for false, 1 for true
	int rebindEveryFrame = 0; // 0 for false, 1 for true
	int perpendicularBranch = 0; // 0 for false, 1 for true
};


class FileParser {
public:
	static std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> extractEdgeTransformsToml(const std::string& filename);
	static std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> extractEdgeTransformsTxt(const std::string& filename);
};

struct PathsConfig
{
	std::string inputFileTxt;
	std::string inputFileToml;

	std::string fragShader;       
	std::string vertShader;  
	std::string newBranchParam;    
	std::string simParam;       

	bool load(const std::string& filename);
	static PathsConfig& get();
};
