#ifndef CAMERA_H
#define CAMERA_H

#include "OpenGLContext.h"

// Std. Includes
#include <vector>
#include <iostream>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};

enum Posture_Adjustment {
	DRAG,
	SCROLL
};

// Default camera values
const GLfloat YAW = -90.0f;
const GLfloat PITCH = 0.0f;
const GLfloat SPEED = 0.5f;
const GLfloat SENSITIVTY = 0.5f;
const GLfloat ZOOM = 45.0f;


// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
	//static int CameraNo;
	// Camera Attributes
	glm::vec3 Position;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	// Eular Angles
	GLfloat Yaw;
	GLfloat Pitch;
	// Camera options
	GLfloat MovementSpeed;
	GLfloat MouseSensitivity;
	GLfloat Zoom;

	// Constructor with vectors
	Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
		GLfloat yaw = YAW, GLfloat pitch = PITCH) :
		Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
	{
		//CameraNo++;
		this->Position = position;
		this->WorldUp = up;
		this->Yaw = yaw;
		this->Pitch = pitch;
		this->updateCameraVectors();
	}
	// Constructor with scalar values
	Camera(GLfloat posX, GLfloat posY, GLfloat posZ,
		GLfloat upX, GLfloat upY, GLfloat upZ,
		GLfloat yaw, GLfloat pitch) :
		Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM)
	{
		this->Position = glm::vec3(posX, posY, posZ);
		this->WorldUp = glm::vec3(upX, upY, upZ);
		this->Yaw = yaw;
		this->Pitch = pitch;
		this->updateCameraVectors();
	}

	// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
	glm::mat4 GetViewMatrix()
	{
		return glm::lookAt(this->Position, this->Position + this->Front, this->Up);
	}

	// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
	void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
	{
		GLfloat velocity = this->MovementSpeed * deltaTime;
		if (direction == FORWARD)
			this->Position += this->Front * velocity;
		if (direction == BACKWARD)
			this->Position -= this->Front * velocity;
		if (direction == LEFT)
			this->Position -= this->Right * velocity;
		if (direction == RIGHT)
			this->Position += this->Right * velocity;
	}

	// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
	void ProcessMouseMovement(GLfloat scrollX, GLfloat scrollY, GLboolean constrainPitch = true)
	{
		scrollX *= this->MouseSensitivity;
		scrollY *= this->MouseSensitivity;

		this->Yaw += scrollX;
		this->Pitch += scrollY;

		//std::cout << "Camera #" << Camera::CameraNo << " deltax = " << scrollX << " deltay = " << scrollY << std::endl;

		// Make sure that when pitch is out of bounds, screen doesn't get flipped
		if (constrainPitch)
		{
			if (this->Pitch > 89.0f)
				this->Pitch = 89.0f;
			if (this->Pitch < -89.0f)
				this->Pitch = -89.0f;
		}

		// Update Front, Right and Up Vectors using the updated Eular angles
		this->updateCameraVectors();
	}

	// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
	void ProcessMouseScroll(GLfloat scrollY)
	{
		if (this->Zoom >= 1.0f && this->Zoom <= 45.0f)
			this->Zoom -= scrollY;
		if (this->Zoom <= 1.0f)
			this->Zoom = 1.0f;
		if (this->Zoom >= 45.0f)
			this->Zoom = 45.0f;
	}

private:
	// Calculates the front vector from the Camera's (updated) Eular Angles
	void updateCameraVectors()
	{
		// Calculate the new Front vector
		glm::vec3 front;
		front.x = cos(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
		front.y = sin(glm::radians(this->Pitch));
		front.z = sin(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
		this->Front = glm::normalize(front);
		// Also re-calculate the Right and Up vector
		this->Right = glm::normalize(glm::cross(this->Front, this->WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
		this->Up = glm::normalize(glm::cross(this->Right, this->Front));
	}
};

class FOVControl
{
public:
	FOVControl() :
		camera(new Camera(glm::vec3(0.0f, 1.0f, 3.0f), glm::vec3(0.0f, 1.0f, -0.4f)))
	{}

	// Moves/alters the camera positions based on user input
	void keyboard_press(bool const * const keyMask)
	{
		if (keyMask[GLFW_KEY_W])
		{
			camera->ProcessKeyboard(FORWARD, deltaTime);
		}
		else if (keyMask[GLFW_KEY_S])
		{
			camera->ProcessKeyboard(BACKWARD, deltaTime);
		}
		else if (keyMask[GLFW_KEY_A])
		{
			camera->ProcessKeyboard(LEFT, deltaTime);
		}
		else if (keyMask[GLFW_KEY_D])
		{
			camera->ProcessKeyboard(RIGHT, deltaTime);
		}
	}

	void move_mouse(GLfloat xpos, GLfloat ypos, GLfloat lastx, GLfloat lasty)
	{
		GLfloat scrollX = xpos - lastx;
		GLfloat scrollY = -(ypos - lasty);
		camera->ProcessMouseMovement(scrollX, scrollY);
	}

	void scroll_mouse(GLfloat scrollX, GLfloat scrollY)
	{
		camera->ProcessMouseScroll(scrollY);
	}

	Camera * getCamera() const
	{
		return this->camera;
	}

private:
	Camera * const camera;
	GLfloat deltaTime = 0.2f;
};

//class FPSCamera : public Camera
//{
//public:
//	// Constructor with vectors
//	FPSCamera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
//		glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
//		GLfloat yaw = YAW, GLfloat pitch = PITCH) :
//		Camera(position, up, yaw) {}
//
//	// Constructor with scalar values
//	FPSCamera(GLfloat posX, GLfloat posY, GLfloat posZ,
//		GLfloat upX, GLfloat upY, GLfloat upZ,
//		GLfloat yaw, GLfloat pitch) :
//		Camera(posX, posY, posZ, upX, upY, upZ, yaw, pitch) {}
//
//	// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
//	void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
//	{
//		GLfloat velocity = this->MovementSpeed * deltaTime;
//		if (direction == FORWARD)
//			this->Position += this->Front * velocity;
//		if (direction == BACKWARD)
//			this->Position -= this->Front * velocity;
//		if (direction == LEFT)
//			this->Position -= this->Right * velocity;
//		if (direction == RIGHT)
//			this->Position += this->Right * velocity;
//		this->Position[1] = 0.0f;
//	}
//};

#endif