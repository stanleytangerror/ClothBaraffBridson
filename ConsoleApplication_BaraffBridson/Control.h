#ifndef GLFW_CONTROL
#define GLFW_CONTROL

// GLEW
//#ifndef GLEW_INCLUDED
//#define GLEW_STATIC
#include <GL/glew.h>
//#define GLEW_INCLUDED
//#endif


// GLFW
#include <GLFW/glfw3.h>
#include "Camera.h"

extern Camera camera;
extern bool keys[];
extern GLfloat lastX, lastY;
extern bool firstMouse;
extern GLfloat deltaTime;
extern GLfloat lastFrame;


// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void mouse_callback(GLFWwindow* window, double xpos, double ypos);

void Do_Movement();

GLuint generateAttachmentTexture(GLboolean depth, GLboolean stencil);

#endif