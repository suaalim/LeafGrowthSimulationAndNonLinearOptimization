#include "Camera.h"

Camera::Camera(glm::vec3 startPos, glm::vec3 startTarget)
	: position(startPos),
	target(startTarget),
	up(0.0f, 1.0f, 0.0f),
	fov(45.0f),
	r(glm::length(position - target)),
	phi(0.0f),
	theta(0.0f)
{
}

void Camera::updateCameraPosition() {
	position.x = r * sin(glm::radians(theta)) * cos(glm::radians(phi));
	position.y = r * sin(glm::radians(theta)) * sin(glm::radians(phi));
	position.z = r * cos(glm::radians(theta));
}

glm::mat4 Camera::getViewMatrix() {
	glm::vec3 eye = glm::normalize(target - position);

	glm::vec3 right =
		glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), eye));

	glm::vec3 up =
		glm::cross(eye, right);

	return glm::lookAt(position, target, up);
}

glm::mat4 Camera::getProjectionMatrix(float aspectRatio) {
	return glm::perspective(
		glm::radians(fov),
		aspectRatio,
		0.1f,
		100.0f
	);
}

glm::mat4 Camera::getOrthoMatrix(float aspectRatio) {
	float orthoWidth = orthoRight - orthoLeft;
	float orthoHeight = orthoTop - orthoBottom;

	float halfW = orthoWidth / 2.0f;
	float halfH = orthoHeight / 2.0f;

	return glm::ortho(
		-halfW * aspectRatio,
		halfW * aspectRatio,
		-halfH,
		halfH,
		orthoNear,
		orthoFar
	);
}

void Camera::processOrthoZoom(float zoomAmount) {
	orthoLeft += zoomAmount;
	orthoRight -= zoomAmount;
	orthoBottom += zoomAmount;
	orthoTop -= zoomAmount;
}

void Camera::processScroll(float yOffset) {
	float zoomSpeed = 0.1f;
	float zoomAmount = yOffset * zoomSpeed;

	processOrthoZoom(zoomAmount);
}

void Camera::moveCamera(glm::vec3 direction) {
	position -= direction;
	target -= direction;
}
