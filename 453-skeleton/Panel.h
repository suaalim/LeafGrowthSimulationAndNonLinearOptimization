#pragma once
#include <glm/glm.hpp>
#include <vector>
#include "Geometry.h"
#include "CurveControl.h" // for ContourBinding

void updateMarkerKeys(const std::vector<ContourBinding>& bindings,
	int stride,
	std::vector<size_t>& markerKeys);

ContourMarkerResult buildContourMarkers(const CPU_Geometry& contourGeometry,
	std::vector<ContourBinding>& bindings,
	const std::vector<size_t>& markerKeys,
	float radius = 0.03f);	

struct SphereMesh {
	std::vector<glm::vec3> verts;   // local-frame, centered at origin
	std::vector<glm::vec3> normals; // local-frame normals (unit length)
	std::vector<unsigned int> indices;
};
