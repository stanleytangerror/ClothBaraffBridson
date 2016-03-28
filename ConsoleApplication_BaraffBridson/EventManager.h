#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include "OpenGLContext.h"

#include "Camera.h"

#include <functional>
#include <list>

// TODO
//GLuint generateAttachmentTexture(GLboolean depth, GLboolean stencil);


// event manager of a given GLFW window
class EventManager
{
public:
	EventManager(GLFWwindow * window) :
		targetWindow(window)
	{
		// Set the required callback functions
		glfwSetKeyCallback(targetWindow, EventManager::keycallback_dispatcher);
		glfwSetCursorPosCallback(targetWindow, EventManager::cursorcallback_dispatcher);
		glfwSetScrollCallback(targetWindow, EventManager::scrollcallback_dispatcher);
	}

	void handleEvents()
	{
		if (invokeMouseCallback)
		{
			(mouseEventHandler)(curX, curY, lastX, lastY);
			invokeMouseCallback = false;
		}
		if (invokescrollCallback)
		{
			(scrollEventHandler)(scrollX, scrollY);
			invokescrollCallback = false;
		}
		if (invokeKeyCallback)
		{
			for (auto hdl : keyboardEventHandlers)
			{
				(hdl)(keyboardMask);
			}
			invokeKeyCallback = false;
		}
	}

	void registerKeyboardEventHandler(std::function< void(bool const * const) > handler)
	{
		keyboardEventHandlers.push_back(handler);
	}

	void registerMouseEventHandler(std::function< void(GLfloat, GLfloat, GLfloat, GLfloat) > handler)
	{
		mouseEventHandler = handler;
	}

	void registerScrollEventHandler(std::function< void(GLfloat, GLfloat) > handler)
	{
		scrollEventHandler = handler;
	}

	// WARNING: should be called before callbacking
	static void active(EventManager * eventManager)
	{
		EventManager::activeEventManager = eventManager;
	}

private:

	// events' target window
	GLFWwindow * const targetWindow;

	// keyboard event 
	bool keyboardMask[1024] = { false };
	std::list<std::function< void(bool const * const) > > keyboardEventHandlers;
	
	// mouse event
	GLfloat lastX = 0.0f, lastY = 0.0f, curX = 0.0f, curY = 0.0f, offsetX = 0.0f, offsetY = 0.0f;
	bool firstMove = true;
	std::function< void(GLfloat, GLfloat, GLfloat, GLfloat) > mouseEventHandler;

	// scroll event
	GLfloat scrollX = 0.0f, scrollY = 0.0f;
	std::function< void(GLfloat, GLfloat) > scrollEventHandler;

	// Function prototypes of GLFW
	void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
	{
		//cout << key << endl;
		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
			glfwSetWindowShouldClose(window, GL_TRUE);
		if (key >= 0 && key < 1024)
		{
			if (action == GLFW_PRESS)
				keyboardMask[key] = true;
			else if (action == GLFW_RELEASE)
				keyboardMask[key] = false;
		}
	}

	// Function prototypes of GLFW
	void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
	{
		this->scrollX = scrollX;
		this->scrollY = scrollY;
	}

	// Function prototypes of GLFW
	void mouse_callback(GLFWwindow* window, double xpos, double ypos)
	{
		//std::cout << "mouse callback" << std::endl;
		if (firstMove)
		{
			lastX = xpos;
			lastY = ypos;
			firstMove = false;
		}
		lastX = curX;
		lastY = curY;

		curX = GLfloat(xpos);
		curY = GLfloat(ypos);
		//offsetX = curX - lastX;
		//offsetY = lastY - curY;  // Reversed since y-coordinates go from bottom to left

	}

	/* NOTE: wrap non-static member functions as static functions
	 * match the parameter list of glfw callbacks
	 * see http://stackoverflow.com/questions/21799746/how-to-glfwsetkeycallback-for-different-classes
	 */
	static EventManager * activeEventManager;

	static bool invokeKeyCallback;
	static bool invokeMouseCallback;
	static bool invokescrollCallback;

	static void keycallback_dispatcher(GLFWwindow *window, int key, int scancode, int action, int mods);
	
	static void cursorcallback_dispatcher(GLFWwindow* window, double xpos, double ypos);

	static void scrollcallback_dispatcher(GLFWwindow* window, double xoffset, double yoffset);

};


#endif