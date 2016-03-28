#include "Screen.h"

// screen properties
GLuint const Screen::screenWidth = 800;
GLuint const Screen::screenHeight = 600;
GLfloat const Screen::aspectRatio = GLfloat(Screen::screenWidth) / GLfloat(Screen::screenHeight);
// render window
GLFWwindow * Screen::window = nullptr;

void Screen::envInfo()
{
	std::cout << "INFO::ENV: OpenGL version " << glGetString(GL_VERSION) << std::endl;
	std::cout << "INFO::ENV: sizeof GLfloat " << sizeof GLfloat << std::endl;
	std::cout << "INFO::ENV: sizeof GLuint " << sizeof GLuint << std::endl;
}

void Screen::initEnv()
{
	initGlfwEnv();
	initGlewEnv();
	initGLOptions();
	envInfo();
}

void Screen::initGLOptions()
{
	// Setup some OpenGL options
	glEnable(GL_DEPTH_TEST);

}

void Screen::initGlfwEnv()
{
	// Init GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	window = glfwCreateWindow(screenWidth, screenHeight, "Scene", nullptr, nullptr); // Windowed
	glfwMakeContextCurrent(window);

	// Options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Define the viewport dimensions
	glViewport(0, 0, screenWidth, screenHeight);

}

void Screen::initGlewEnv()
{
	// Initialize GLEW to setup the OpenGL Function pointers
	glewExperimental = GL_TRUE;
	glewInit();
}

void Screen::endGlfw()
{
	glfwTerminate();
}
