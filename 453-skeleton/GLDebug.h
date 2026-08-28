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
	float getMinRebindContourDistance() const { return minRebindContourDistance; }
	float getMaxRebindContourDistance() const{ return maxRebindContourDistance; }
	float getDeltaTime() const { return deltaTime; }
	int getPointsPerSegment() const { return pointsPerSegment; }
	int getBestBindingStep() const { return bestBindingStep; }
	int getsubdivideBranch() const { return subdivideBranch; }
	int getRebindEveryFrame() const { return rebindEveryFrame; }
	int getPerpendicularBranch() const { return perpendicularBranch; }
	int getBlend() const { return blend;  }
	float getIndentationWindow() const { return indentationWindow; }
	float getNewBranchDistance() const { return newBranchDistance; }
	float getNewBranchExclusion() const { return newBranchExclusion; }
	int getPetiole() const { return petiole; }
	int getSymmetricBranch() const { return symmetricBranch; }
	int getPerpendicularBinding() const { return perpendicularBinding; }
	float getGrowthReductionThreshold() const { return growthReductionThreshold; }
	float getGrowthReductionFactor() const { return growthReductionFactor; }
	float getGrowthReductionStep() const { return growthReductionStep; }
	int getTipToTipBlending() const { return tipToTipBlending; }

private:
	SimulationConfig() = default;

	float subdivisionThreshold = 0.0f;
	float subdivisionDivision = 0.0f;
	float newContourPointThreshold = 0.0f;
	float newBindingPointThreshold = 0.0f;
	float minRebindContourDistance = 0.0f;
	float maxRebindContourDistance = 0.0f;
	float deltaTime = 0.0f;
	int pointsPerSegment = 0;
	int bestBindingStep = 0;
	int subdivideBranch = 0; // 0 for false, 1 for true
	int rebindEveryFrame = 0; // 0 for false, 1 for true
	int perpendicularBranch = 0; // 0 for false, 1 for true
	int blend = 0; // 0 for false, 1 for true
	float indentationWindow = 0.0f;
	float newBranchDistance = 0.f;
	float newBranchExclusion = 0.f;
	int petiole = 0; // 0 for false, 1 for true
	int symmetricBranch = 0; // 0 for false, 1 for true
	int perpendicularBinding = 0; // 0 for false, 1 for true
	float growthReductionThreshold = 0.f;
	float growthReductionStep = 0.f;
	float growthReductionFactor = 0.f;
	int tipToTipBlending = 0; // 0 for false, 1 for true
};

class PetioleConfig
{
public:
	static PetioleConfig& instance()
	{
		static PetioleConfig inst;
		return inst;
	}

	bool load(const std::string& filename);

	float getBranchingNodeS() const { return branchingNodeS; }
	float getBranchingNodeExpansion() const { return branchingNodeExpansion; }
	float getBranchingNodeGrowth() const { return branchingNodeGrowth; }
	float getNewBranchPetioleEndS() const { return newBranchPetioleEndS; }
	float getNewBranchPetioleEndExpansion() const { return newBranchPetioleEndExpansion; }
	float getNewBranchPetioleEndGrowth() const { return newBranchPetioleEndGrowth; }
	float getNewBranchPetioleAfterS() const { return newBranchPetioleAfterS; }
	float getNewBranchPetioleAfterExpansion() const { return newBranchPetioleAfterExpansion; }
	float getNewBranchPetioleAfterGrowth() const { return newBranchPetioleAfterGrowth; }
	float getMainAxisPetioleEndS() const { return mainAxisPetioleEndS; }
	float getMainAxisPetioleEndExpansion() const { return mainAxisPetioleEndExpansion; }
	float getMainAxisPetioleEndGrowth() const { return mainAxisPetioleEndGrowth; }
	float getMainAxisPetioleAfterS() const { return mainAxisPetioleAfterS; }
	float getMainAxisPetioleAfterExpansion() const { return mainAxisPetioleAfterExpansion; }
	float getMainAxisPetioleAfterGrowth() const { return mainAxisPetioleAfterGrowth; }
	float getNewBranchDivision1() const { return newBranchDivision1; }
	float getNewBranchDivision2() const { return newBranchDivision2; }
	float getNewBranchDivision3() const { return newBranchDivision3; }
	float getMainAxisDivision1() const { return mainAxisDivision1; }
	float getMainAxisDivision2() const { return mainAxisDivision2; }
	float getMainAxisDivision3() const { return mainAxisDivision3; }

private:
	PetioleConfig() = default;

	float branchingNodeS = 0.0f;
	float branchingNodeExpansion = 0.0f;
	float branchingNodeGrowth = 0.0f;
	float newBranchPetioleEndS = 0.0f;
	float newBranchPetioleEndExpansion = 0.0f;
	float newBranchPetioleEndGrowth = 0.0f;
	float newBranchPetioleAfterS = 0.0f;
	float newBranchPetioleAfterExpansion = 0.0f;
	float newBranchPetioleAfterGrowth = 0.0f;
	float mainAxisPetioleEndS = 0.0f; 
	float mainAxisPetioleEndExpansion = 0.0f; 
	float mainAxisPetioleEndGrowth = 0.0f; 
	float mainAxisPetioleAfterS = 0.0f; 
	float mainAxisPetioleAfterExpansion = 0.0f;
	float mainAxisPetioleAfterGrowth = 0.f;
	float newBranchDivision1 = 0.0f;
	float newBranchDivision2 = 0.0f;
	float newBranchDivision3 = 0.0f;
	float mainAxisDivision1 = 0.0f;
	float mainAxisDivision2 = 0.0f;
	float mainAxisDivision3 = 0.0f;
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
	std::string petioleParam;

	bool load(const std::string& filename);
	static PathsConfig& get();
};
