#include "GLDebug.h"
#include "toml.hpp"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <regex>

bool NewBranchConfig::load(const std::string& filename)
{
	try
	{
		auto tbl = toml::parse_file(filename);

		newBranchS =
			static_cast<float>(tbl["new_branch_S"].value_or(0.0));

		newBranchExpansionFactor =
			static_cast<float>(tbl["new_branch_expansion_factor"].value_or(0.0));

		newBranchGrowthFactor =
			static_cast<float>(tbl["new_branch_growth_factor"].value_or(0.0));

		return true;
	}
	catch (const toml::parse_error& err)
	{
		std::cerr << "TOML parse error: "
			<< err.description()
			<< std::endl;
		return false;
	}
}

bool SimulationConfig::load(const std::string& filename)
{
	try
	{
		auto tbl = toml::parse_file(filename);

		subdivisionThreshold =
			static_cast<float>(tbl["subdivision_threshold"].value_or(0.0));

		subdivisionDivision =
			static_cast<float>(tbl["subdivision_division"].value_or(0.0));

		newContourPointThreshold =
			static_cast<float>(tbl["new_contour_point_threshold"].value_or(0.0));

		newBindingPointThreshold =
			static_cast<float>(tbl["new_binding_point_threshold"].value_or(0.0));

		rebindContourDistance =
			static_cast<float>(tbl["rebind_contour_to_new_branch_distance"].value_or(0.0));

		deltaTime =
			static_cast<float>(tbl["delta_time"].value_or(0.0));

		pointsPerSegment =
			static_cast<int>(tbl["points_per_segment"].value_or(0));

		bestBindingStep = 
			static_cast<int>(tbl["best_binding_steps"].value_or(0));

		subdivideBranch =
			static_cast<int>(tbl["subdivide_branch"].value_or(0));

		rebindEveryFrame =
			static_cast<int>(tbl["rebind_every_frame"].value_or(0));

		perpendicularBranch =
			static_cast<int>(tbl["perpendicular_branch"].value_or(0));

		return true;
	}
	catch (const toml::parse_error& err)
	{
		std::cerr << "TOML parse error: "
			<< err.description()
			<< std::endl;
		return false;
	}
}

// ------------ toml parser -------------------
glm::mat4 parseMat4Toml(const toml::array* arr)
{
	glm::mat4 m(1.0f);
	int row = 0;
	for (auto&& r : *arr)
	{
		const auto& rowArr = *r.as_array();
		int col = 0;
		for (auto&& v : rowArr)
		{
			m[col][row] = static_cast<float>(v.as_floating_point()->get()); // col-major: m[col][row]
			col++;
		}
		row++;
	}
	return m;
}

std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> FileParser::extractEdgeTransformsToml(const std::string& filename)
{
	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> edges;

	toml::table tbl;

	try {
		tbl = toml::parse_file(filename);
	}
	catch (const toml::parse_error& err) {
		std::cerr << "TOML parse error: " << err.description() << "\n";
		return {};
	}

	auto edgeArray = tbl["edge"].as_array();
	if (!edgeArray) {
		std::cerr << "No [edge] entries found\n";
		return {};
	}

	int branchID = 0;
	int axisID = 0;
	for (auto&& edgeVal : *edgeArray)
	{
		auto& edge = *edgeVal.as_table();

		// branch = [parent, child]
		auto branch = *edge["branch"].as_array();
		int parent = static_cast<int>(branch[0].as_integer()->get());
		int child = static_cast<int>(branch[1].as_integer()->get());

		glm::mat4 rotation = parseMat4Toml(edge["rotation"].as_array());
		glm::mat4 scaling = parseMat4Toml(edge["scaling"].as_array());
		glm::mat4 translation = parseMat4Toml(edge["translation"].as_array());

		float scalingFactor = static_cast<float>(edge["scaling_factor"].as_floating_point()->get());
		int rotationDirection = static_cast<int>(edge["rotation_direction"].as_integer()->get());
		float rotationAngle = static_cast<float>(edge["rotation_angle"].as_floating_point()->get());
		float expansionFactor = static_cast<float>(edge["expansion_factor"].as_floating_point()->get());
		float growthFactor = static_cast<float>(edge["growth_factor"].as_floating_point()->get());
		float branchPosition = static_cast<float>(edge["branch_position"].as_floating_point()->get());

		edges.emplace_back(
			parent, child,
			rotation, scaling, translation,
			scalingFactor, rotationDirection,
			rotationAngle, expansionFactor,
			growthFactor, branchPosition,
			axisID, branchID
		);
		axisID++;
	}

	return edges;
}


// ------------ txt parser --------------------
glm::mat4 parseMatrix(std::ifstream& in) {
	glm::mat4 mat(1.0f);
	for (int i = 0; i < 4; i++) {
		std::string line;
		std::getline(in, line);
		std::stringstream ss(line);
		for (int j = 0; j < 4; j++) {
			ss >> mat[j][i]; // column-major order
		}
	}
	return mat;
}

// to build initial branching structure from the python file
// extract the local matrices per edge
std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> FileParser::extractEdgeTransformsTxt(const std::string& filename) {
	std::ifstream in(filename);
	if (!in.is_open()) {
		std::cerr << "Failed to open file\n";
		return {};
	}

	std::vector<std::tuple<int, int, glm::mat4, glm::mat4, glm::mat4, float, int, float, float, float, float, int, int>> edges;
	std::string line;
	std::regex edgeRegex(R"#(Edge\s+(\d+)\s+->\s+(\d+))#");
	std::smatch match;

	// only has information for children! Root is just identity
	// so need to start ID from 1 (0 for root)
	int bID = 0;
	int aID = 0;

	while (std::getline(in, line)) {
		if (std::regex_search(line, match, edgeRegex)) {
			int parent = std::stoi(match[1]);
			int child = std::stoi(match[2]);

			std::getline(in, line); // skip header
			std::getline(in, line);
			glm::mat4 rotation = parseMatrix(in);
			std::getline(in, line);
			glm::mat4 scaling = parseMatrix(in);
			std::getline(in, line);
			glm::mat4 translation = parseMatrix(in);
			std::getline(in, line);
			std::getline(in, line); // scaling factor
			float scalingFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // rotation direction
			float rotationDirection = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // rotation angle
			float rotationAngle = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // expansion factor
			float expansionFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // growth factor
			float growthFactor = std::stof(line);
			std::getline(in, line);
			std::getline(in, line); // position on branch
			float branchPosition = std::stof(line);

			edges.emplace_back(parent, child, rotation, scaling, translation, scalingFactor, rotationDirection, rotationAngle, expansionFactor, growthFactor, branchPosition, aID, bID);
			aID++;
		}
	}

	return edges;
}

bool PathsConfig::load(const std::string& filename)
{
	try
	{
		auto tbl = toml::parse_file(filename);
		auto paths = tbl["paths"];

		inputFileTxt = paths["inputFileTxt"].value_or(std::string(""));
		inputFileToml = paths["inputFileToml"].value_or(std::string(""));
		fragShader = paths["fragShader"].value_or(std::string(""));
		vertShader = paths["vertShader"].value_or(std::string(""));
		newBranchParam = paths["newBranchParam"].value_or(std::string(""));
		simParam = paths["simParam"].value_or(std::string(""));

		return true;
	}
	catch (const toml::parse_error& err)
	{
		std::cerr << "TOML parse error: "
			<< err.description()
			<< std::endl;
		return false;
	}
}

PathsConfig& PathsConfig::get()
{
	static PathsConfig instance;
	return instance;
}
