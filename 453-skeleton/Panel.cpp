#include "panel.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>

static glm::vec3 hueToRGB(float hue01) {
	float h = hue01 * 6.0f;
	float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
	glm::vec3 c;
	if (h < 1.0f)      c = { 1, x, 0 };
	else if (h < 2.0f) c = { x, 1, 0 };
	else if (h < 3.0f) c = { 0, 1, x };
	else if (h < 4.0f) c = { 0, x, 1 };
	else if (h < 5.0f) c = { x, 0, 1 };
	else               c = { 1, 0, x };
	return c;
}

CPU_Geometry generateCircle(float radius, int segments = 16) {
	CPU_Geometry circle;
	circle.verts.reserve(segments);
	circle.cols.reserve(segments);
	circle.indices.reserve(segments * 2);

	for (int i = 0; i < segments; i++) {
		float angle = 2.0f * glm::pi<float>() * float(i) / float(segments);
		circle.verts.push_back(glm::vec3(radius * cos(angle), radius * sin(angle), 0.0f)); // XY plane
		circle.cols.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
		circle.indices.push_back(i);
		circle.indices.push_back((i + 1) % segments);
	}
	return circle;
}

SphereMesh generateSphere(float radius, int stacks, int slices) {
	SphereMesh sphere;

	for (int i = 0; i <= stacks; ++i) {
		float phi = glm::pi<float>() * (float)i / (float)stacks;
		float y = std::cos(phi);
		float ringRadius = std::sin(phi);

		for (int j = 0; j <= slices; ++j) {
			float theta = 2.0f * glm::pi<float>() * (float)j / (float)slices;
			float x = ringRadius * std::cos(theta);
			float z = ringRadius * std::sin(theta);

			glm::vec3 normal = glm::normalize(glm::vec3(x, y, z));
			sphere.verts.push_back(normal * radius); // centered at local origin
			sphere.normals.push_back(normal);
		}
	}

	int vertsPerRing = slices + 1;
	for (int i = 0; i < stacks; ++i) {
		for (int j = 0; j < slices; ++j) {
			unsigned int a = i * vertsPerRing + j;
			unsigned int b = a + vertsPerRing;
			unsigned int c = a + 1;
			unsigned int d = b + 1;

			sphere.indices.push_back(a);
			sphere.indices.push_back(b);
			sphere.indices.push_back(c);

			sphere.indices.push_back(c);
			sphere.indices.push_back(b);
			sphere.indices.push_back(d);
		}
	}

	return sphere;
}

void updateMarkerKeys(const std::vector<ContourBinding>& bindings, int stride, std::vector<size_t>& markerKeys) {
	//size_t n = bindings.size();

	//while (true) {    
	//	size_t targetIndex = markerKeys.size() + (size_t)stride;
	//	if (targetIndex >= n) break;
	//	markerKeys.push_back(bindings[targetIndex].uniqueKey);  // store contour points (key) to draw sphere
	//}
	markerKeys.push_back(bindings[11].uniqueKey);
	markerKeys.push_back(bindings[10].uniqueKey);
	markerKeys.push_back(bindings[6].uniqueKey);
	markerKeys.push_back(bindings[5].uniqueKey);
}

ContourMarkerResult buildContourMarkers(const CPU_Geometry& contourGeometry, std::vector<ContourBinding>& bindings, const std::vector<size_t>& markerKeys, float radius) {
	ContourMarkerResult result;
	SphereMesh localSphere = generateSphere(radius, 8, 8);  // increase resolution if necessary
	CPU_Geometry localCircle = generateCircle(radius, 32);

	std::unordered_map<size_t, size_t> keyToIndex;
	keyToIndex.reserve(bindings.size());
	for (size_t idx = 0; idx < bindings.size(); idx++) {
		keyToIndex[bindings[idx].uniqueKey] = idx;  // record where each uniqueKey lives (index) -> updated every frame
	}

	const float normalLength = radius * 0.3f;

	for (size_t key : markerKeys) {
		auto it = keyToIndex.find(key);  // find current index for uniqueKey (since it changes every frame)
		if (it == keyToIndex.end()) {
			continue; 
		}
		size_t i = it->second; // i = index of uniqueKey
		if (i >= contourGeometry.verts.size()) {
			continue;
		}

		glm::vec3 originalPos = contourGeometry.verts[i];
		const glm::mat4 transform =
			(bindings[i].t) * bindings[i].childNode->marginTransformation +
			(1 - bindings[i].t) * bindings[i].parentNode->marginTransformation *
			glm::toMat4(bindings[i].childNode->localRotation);

		glm::mat3 normalMatrix = glm::inverseTranspose(glm::mat3(transform));

		CPU_Geometry sphere;
		sphere.indices = localSphere.indices;
		sphere.verts.reserve(localSphere.verts.size());
		sphere.cols.reserve(localSphere.verts.size());

		for (size_t v = 0; v < localSphere.verts.size(); v++) {
			glm::vec4 worldPos = transform * glm::vec4(localSphere.verts[v], 1.0f);  // transform each vertex of the sphere
			sphere.verts.push_back(glm::vec3(worldPos));

			glm::vec3 normalShade = localSphere.normals[v] * 0.5f + 0.5f;  // use normals to color the sphere (do not transform the normals so that we see rotation)
			sphere.cols.push_back(normalShade);

			// to draw normals
			glm::vec3 worldNormal = glm::normalize(normalMatrix * localSphere.normals[v]);
			unsigned int lineOffset = (unsigned int)result.normals.verts.size();
			result.normals.verts.push_back(glm::vec3(worldPos));
			result.normals.verts.push_back(glm::vec3(worldPos) + worldNormal * normalLength);
			result.normals.cols.push_back(glm::vec3(1, 0, 0));
			result.normals.cols.push_back(glm::vec3(1, 0, 0));
			result.normals.indices.push_back(lineOffset);
			result.normals.indices.push_back(lineOffset + 1);
		}

		glm::vec3 transformedCenter = glm::vec3(transform * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

		unsigned int indexOffset = (unsigned int)result.spheres.verts.size();
		result.spheres.verts.insert(result.spheres.verts.end(), sphere.verts.begin(), sphere.verts.end());
		result.spheres.cols.insert(result.spheres.cols.end(), sphere.cols.begin(), sphere.cols.end());
		for (unsigned int idx : sphere.indices) {
			result.spheres.indices.push_back(idx + indexOffset);
		}

		unsigned int lineOffset = (unsigned int)result.connectors.verts.size();
		result.connectors.verts.push_back(originalPos);
		result.connectors.verts.push_back(transformedCenter);
		result.connectors.cols.push_back(glm::vec3(0, 0, 1));
		result.connectors.cols.push_back(glm::vec3(0, 0, 1));
		result.connectors.indices.push_back(lineOffset);
		result.connectors.indices.push_back(lineOffset + 1);

		unsigned int circleOffset = (unsigned int)result.referenceCircles.verts.size();
		for (size_t v = 0; v < localCircle.verts.size(); v++) {
			result.referenceCircles.verts.push_back(localCircle.verts[v] + transformedCenter); // use transformed center to draw it on top of the sphere after being transformed
			result.referenceCircles.cols.push_back(localCircle.cols[v]);
		}
		for (unsigned int idx : localCircle.indices) {
			result.referenceCircles.indices.push_back(idx + circleOffset);
		}
	}

	return result;
}

//ContourMarkerResult buildContourMarkers(const CPU_Geometry& contourGeometry,
//	std::vector<ContourBinding>& bindings,
//	int stride,
//	float radius) {
//	ContourMarkerResult result;
//	size_t n = contourGeometry.verts.size();
//	size_t markerCount = (n + stride - 1) / stride;
//
//	// Generate the local-frame unit sphere once; reuse for every marker
//	SphereMesh localSphere = generateSphere(radius, 8, 8);
//
//	size_t markerIdx = 0;
//	for (size_t i = 0; i < n; i += stride, markerIdx++) {
//		glm::vec3 originalPos = contourGeometry.verts[i];
//		glm::vec3 tint = hueToRGB((float)markerIdx / (float)markerCount);
//
//		const glm::mat4 transform =
//			(bindings[i].t) * bindings[i].childNode->marginTransformation +
//			(1 - bindings[i].t) * bindings[i].parentNode->marginTransformation *
//			glm::toMat4(bindings[i].childNode->localRotation);
//
//		glm::mat3 rotationPart = glm::mat3(transform);
//
//		CPU_Geometry sphere;
//		sphere.indices = localSphere.indices;
//		sphere.verts.reserve(localSphere.verts.size());
//		sphere.cols.reserve(localSphere.verts.size());
//
//		for (size_t v = 0; v < localSphere.verts.size(); ++v) {
//			// Apply transform to the LOCAL-frame vertex (origin-centered)
//			glm::vec4 worldPos = transform * glm::vec4(localSphere.verts[v], 1.0f);
//			sphere.verts.push_back(glm::vec3(worldPos));
//
//			// Rotate the normal by the rotation part only, then renormalize
//			glm::vec3 worldNormal = glm::normalize(rotationPart * localSphere.normals[v]);
//			glm::vec3 normalShade = worldNormal * 0.5f + 0.5f;
//			//sphere.cols.push_back(glm::mix(normalShade, tint, 0.7f));
//			sphere.cols.push_back(normalShade);
//		}
//
//		// Where the marker itself (the point, not just its shell) ends up
//		glm::vec3 transformedCenter = glm::vec3(transform * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
//
//		// Merge sphere
//		unsigned int indexOffset = (unsigned int)result.spheres.verts.size();
//		result.spheres.verts.insert(result.spheres.verts.end(), sphere.verts.begin(), sphere.verts.end());
//		result.spheres.cols.insert(result.spheres.cols.end(), sphere.cols.begin(), sphere.cols.end());
//		for (unsigned int idx : sphere.indices) {
//			result.spheres.indices.push_back(idx + indexOffset);
//		}
//
//		// Connector: original contour point -> where the transform sent it
//		unsigned int lineOffset = (unsigned int)result.connectors.verts.size();
//		result.connectors.verts.push_back(originalPos);
//		result.connectors.verts.push_back(transformedCenter);
//		result.connectors.cols.push_back(tint);
//		result.connectors.cols.push_back(tint);
//		result.connectors.indices.push_back(lineOffset);
//		result.connectors.indices.push_back(lineOffset + 1);
//	}
//
//	/*
//	for (size_t i = 0; i < n; i += stride, markerIdx++) {
//		glm::vec3 originalPos = contourGeometry.verts[i];
//
//		glm::vec3 tint = hueToRGB((float)markerIdx / (float)markerCount);
//
//		CPU_Geometry sphere = generateSphere(originalPos, radius, 8, 8, tint);
//
//		unsigned int indexOffset = (unsigned int)result.spheres.verts.size();
//		result.spheres.verts.insert(result.spheres.verts.end(),
//			sphere.verts.begin(),
//			sphere.verts.end());
//		result.spheres.cols.insert(result.spheres.cols.end(),
//			sphere.cols.begin(),
//			sphere.cols.end());
//
//		for (unsigned int idx : sphere.indices) {
//			result.spheres.indices.push_back(idx + indexOffset);
//		}
//	}
//	*/
//
//	return result;
//}
