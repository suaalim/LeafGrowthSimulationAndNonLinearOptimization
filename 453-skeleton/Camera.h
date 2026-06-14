#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
	glm::vec3 position;
	glm::vec3 target;
	glm::vec3 up;

	float r;
	float theta;
	float phi;
	float fov;

	float orthoLeft = -1.0f;
	float orthoRight = 1.0f;
	float orthoBottom = -1.0f;
	float orthoTop = 1.0f;
	float orthoNear = 0.1f;
	float orthoFar = 100.0f;

	Camera(glm::vec3 startPos, glm::vec3 startTarget);

	void updateCameraPosition();

	glm::mat4 getViewMatrix();

	glm::mat4 getProjectionMatrix(float aspectRatio);

	glm::mat4 getOrthoMatrix(float aspectRatio);

	void processOrthoZoom(float zoomAmount);

	void processScroll(float yOffset);

	void moveCamera(glm::vec3 direction);
};
