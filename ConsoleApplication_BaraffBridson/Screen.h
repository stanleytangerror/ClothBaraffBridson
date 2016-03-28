#ifndef SCREEN_H
#define SCREEN_H

#include "OpenGLContext.h"

#include <iostream>

class Screen
{
public:
	// screen properties
	static GLuint const screenWidth, screenHeight;
	static GLfloat const aspectRatio;
	// render window
	static GLFWwindow * window;

	static void envInfo();

	static void initEnv();

	static void swapBuffers()
	{
		glfwSwapBuffers(window);
	}

	static bool closed()
	{
		return glfwWindowShouldClose(window);
	}

	static void pullEvents()
	{
		//// Set frame time
		//GLfloat currentFrame = (GLfloat)glfwGetTime();
		//deltaTime = currentFrame - lastFrame;
		//lastFrame = currentFrame;

		// Check and call events
		glfwPollEvents();
	}

private:
	Screen() {}

	static void initGLOptions();

	static void initGlfwEnv();

	static void initGlewEnv();

	static void endGlfw();

};

#endif
