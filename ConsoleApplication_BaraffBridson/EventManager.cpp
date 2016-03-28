#include "EventManager.h"
#include <iostream>

EventManager * EventManager::activeEventManager;

bool EventManager::invokeKeyCallback = false;
bool EventManager::invokeMouseCallback = false;
bool EventManager::invokescrollCallback = false;

void EventManager::keycallback_dispatcher(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if (EventManager::activeEventManager)
	{
		EventManager::activeEventManager->key_callback(window, key, scancode, action, mods);
		EventManager::invokeKeyCallback = true;
	}
	else
		std::cout << "ERROR: no active event manager" << std::endl;
	//activeEventManager->keycallback(window, key, scancode, action, mods);
}

void EventManager::cursorcallback_dispatcher(GLFWwindow* window, double xpos, double ypos)
{
	if (EventManager::activeEventManager)
	{
		EventManager::activeEventManager->mouse_callback(window, xpos, ypos);
		EventManager::invokeMouseCallback = true;
	}
	else
		std::cout << "ERROR: no active event manager" << std::endl;
}

void EventManager::scrollcallback_dispatcher(GLFWwindow* window, double xoffset, double yoffset)
{
	if (EventManager::activeEventManager)
	{
		EventManager::activeEventManager->scroll_callback(window, xoffset, yoffset);
		EventManager::invokescrollCallback = true;
	}
	else
		std::cout << "ERROR: no active event manager" << std::endl;
}
