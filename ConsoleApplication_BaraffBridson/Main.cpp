// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GL includes
#include "Shader.h"
#include "Camera.h"
#include "Model.h"
#include "Control.h"
#include "ResourceManager.h"

#include "ClothPiece.h"
#include "Simulate.h"
#include "Test.h"

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Other Libs
#include <SOIL.h>

// Std. Includes
#include <string>
#include <iostream>

// Properties
const GLuint screenWidth = 800, screenHeight = 600;

GLint const init_loop = 0;
// Function prototypes
//void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
//void mouse_callback(GLFWwindow* window, double xpos, double ypos);
//void Do_Movement();

// Camera
//Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
//bool keys[1024];
//GLfloat lastX = 400, lastY = 300;
//bool firstMouse = true;
//
//GLfloat deltaTime = 0.0f;
//GLfloat lastFrame = 0.0f;

const GLfloat backgroundCubeVertices[] = {
	// Positions          
	-1.0f,  1.0f, -1.0f,
	-1.0f, -1.0f, -1.0f,
	1.0f, -1.0f, -1.0f,
	1.0f, -1.0f, -1.0f,
	1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	1.0f, -1.0f, -1.0f,
	1.0f, -1.0f,  1.0f,
	1.0f,  1.0f,  1.0f,
	1.0f,  1.0f,  1.0f,
	1.0f,  1.0f, -1.0f,
	1.0f, -1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	1.0f,  1.0f,  1.0f,
	1.0f,  1.0f,  1.0f,
	1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	-1.0f,  1.0f, -1.0f,
	1.0f,  1.0f, -1.0f,
	1.0f,  1.0f,  1.0f,
	1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	1.0f, -1.0f, -1.0f,
	1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	1.0f, -1.0f,  1.0f
};

const std::string modelPath = "E:\\Computer Graphics\\Materials\\Models\\SquareCloth04\\Clothes.obj";
//const std::string modelPath = "E:/Computer Graphics/Materials/nanosuit/nanosuit.obj";
//const std::string modelPath = "E:\\Computer Graphics\\Materials\\nanosuit\\nanosuit.obj";
//const std::string modelPath = "E:\\Computer Graphics\\Materials\\Huang\\model02\\test01.obj";
//const std::string modelPath = "E:\\Computer Graphics\\Materials\\Huang\\model01\\test.3DS";
//const std::string modelPath = "E:\\Computer Graphics\\Codes\\Xue\\薛原 - 毕业材料整理\\薛原 - 毕业材料整理\\程序代码\\新人体\\3ds读取 - 新人体衣服 - 女 - 紧身\\Debug\\T恤2.3DS";

void basic_info();

// The MAIN function, from here we start our application and run our Game loop
int main()
{
	//testPCG();

	// Init GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "LearnOpenGL", nullptr, nullptr); // Windowed
	glfwMakeContextCurrent(window);

	// Set the required callback functions
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// Options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Initialize GLEW to setup the OpenGL Function pointers
	glewExperimental = GL_TRUE;
	glewInit();

	basic_info();

	// Define the viewport dimensions
	glViewport(0, 0, screenWidth, screenHeight);

	// Setup some OpenGL options
	glEnable(GL_DEPTH_TEST);

	// Setup and compile our shaders
	ResourceManager::LoadShader("model_loading", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.frag");
	ResourceManager::LoadShader("background_cube", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.frag");
	ResourceManager::LoadShader("cloth_piece", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.frag");
	ResourceManager::LoadShader("cloth_piece_debug", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.gs");
	//ResourceManager::LoadShader("model_loading", ".\\model_loading.vs", ".\\model_loading.frag");
	//ResourceManager::LoadShader("background_cube", ".\\background_cube.vs", ".\\background_cube.frag");
	//ResourceManager::LoadShader("cloth_piece", ".\\cloth_piece.vs", ".\\cloth_piece.frag");

	std::vector<const GLchar*> faces;
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\top.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\bottom.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	ResourceManager::LoadCubeMap("background_texture", faces);

	GLuint backgroundVAO;
	glGenVertexArrays(1, &backgroundVAO);
	glBindVertexArray(backgroundVAO);
	{
		GLuint backgroundVBO;
		glGenBuffers(1, &backgroundVBO);
		glBindBuffer(GL_ARRAY_BUFFER, backgroundVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(backgroundCubeVertices), &backgroundCubeVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
	}
	glBindVertexArray(0);

	// Load models
	Model ourModel((GLchar *)modelPath.c_str(), (aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	ClothPiece* clothPiece = new ClothPiece(3);
	clothPiece->import(ourModel.getMeshes()[0]);
	clothPiece->useVTexCoord2DAsVPlanarCoord3f();
	Simulate * simulate = new Simulate(clothPiece);


	GLfloat * meshVB = nullptr;
	GLfloat * meshVNormalB = nullptr;
	GLuint * meshEB = nullptr;
	GLuint meshVBcnt = 0, meshEBcnt = 0;

	GLfloat * conditionBuffer = nullptr;
	GLuint conditionCnt = 0;

	GLuint meshVAO;
	glGenVertexArrays(1, &meshVAO);
	std::cout << "mesh vao " << meshVAO << std::endl;

	GLuint meshVBO, meshVNormalBO, meshEBO;
	GLuint conditionVBO;

	glGenBuffers(1, &meshVBO);
	std::cout << "mesh vbo " << meshVBO << std::endl;
	glGenBuffers(1, &meshVNormalBO);
	std::cout << "mesh vbNormalo " << meshVNormalBO << std::endl;
	glGenBuffers(1, &meshEBO);
	std::cout << "mesh ebo " << meshEBO << std::endl;
	glGenBuffers(1, &conditionVBO);
	std::cout << "condition vbo " << conditionVBO << std::endl;

	GLuint debugVAO;
	glGenVertexArrays(1, &debugVAO);
	std::cout << "debug vao " << debugVAO << std::endl;

	GLuint debugVBO, debugNormalVBO;
	glGenBuffers(1, &debugVBO);
	std::cout << "debug vbo " << debugVBO << std::endl;
	glGenBuffers(1, &debugNormalVBO);
	std::cout << "debug normal vbo " << debugNormalVBO << std::endl;

	// Game loop
	GLint loop_cnt = 0;
	while (!glfwWindowShouldClose(window))
	{
		// Set frame time
		GLfloat currentFrame = (GLfloat) glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Check and call events
		glfwPollEvents();
		Do_Movement();

		// Clear the colorbuffer
		glClearColor(0.6f, 0.6f, 0.6f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 projection = glm::perspective(camera.Zoom, (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();

		// draw background cubemap
		glDepthMask(GL_FALSE);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		{
			Shader backgoroundShader = ResourceManager::GetShader("background_cube");
			backgoroundShader.Use();
			glUniformMatrix4fv(glGetUniformLocation(backgoroundShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(backgoroundShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(glm::mat4(glm::mat3(view))));

			glBindVertexArray(backgroundVAO);
			glActiveTexture(GL_TEXTURE0);
			glUniform1i(glGetUniformLocation(backgoroundShader.Program, "background"), GL_TEXTURE0 - GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_CUBE_MAP, ResourceManager::GetCubeMap("background_texture").ID);
			glDrawArrays(GL_TRIANGLES, 0, 36);
			glBindVertexArray(0);

		}
		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDepthMask(GL_TRUE);


//#define SHOW_MODEL
#define USE_SIMULATION

#ifdef SHOW_MODEL
		{		// use model
			Shader modelShader = ResourceManager::GetShader("model_loading");
			modelShader.Use();   // <-- Don't forget this one!
							// Transformation matrices
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
			glm::mat4 model;
			model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));
			ourModel.Draw(modelShader);
		}
#endif

		// use mesh
#ifdef USE_SIMULATION
		{
			Shader clothPieceShader = ResourceManager::GetShader("cloth_piece");
			clothPieceShader.Use();
			glm::mat4 model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
			model = glm::translate(model, glm::vec3(0.0f, 0.40f, 0.0f)); // Translate it down a bit so it's at the center of the scene
			glUniformMatrix4fv(glGetUniformLocation(clothPieceShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(clothPieceShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(glGetUniformLocation(clothPieceShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "viewPos"), camera.Position.x, camera.Position.y, camera.Position.z);
			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "light.direction"), 0.0f, 1.0f, 0.0f);
			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "light.color"), 0.3f, 0.49f, 0.85f);
			/* from http://devernay.free.fr/cours/opengl/materials.html */
			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "material.ambient"), 0.19225f, 0.19225f, 0.19225f);
			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "material.diffuse"), 0.50754f, 0.50754f, 0.50754f);
			glUniform3f(glGetUniformLocation(clothPieceShader.Program, "material.specular"), 0.508273f, 0.508273f, 0.508273f);
			glUniform1f(glGetUniformLocation(clothPieceShader.Program, "material.shininess"), 0.4f);

			// Draw the loaded model
			if (loop_cnt < init_loop)
			{
				simulate->simulate();
				loop_cnt += 1;
			}
			if (keys[81])
			{
				loop_cnt -= 1;
			}
			simulate->writeBack();
			clothPiece->exportPos3fNorm3fBuffer(meshVB, meshVNormalB, meshVBcnt, meshEB, meshEBcnt);
			simulate->exportShearConditionData(conditionBuffer, conditionCnt);


			glBindVertexArray(meshVAO);
			{
				glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
				glBufferData(GL_ARRAY_BUFFER, meshVBcnt * 3 * sizeof(GLfloat), meshVB, GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
				glEnableVertexAttribArray(0);
				glBindBuffer(GL_ARRAY_BUFFER, 0);

				glBindBuffer(GL_ARRAY_BUFFER, meshVNormalBO);
				glBufferData(GL_ARRAY_BUFFER, meshVBcnt * 3 * sizeof(GLfloat), meshVNormalB, GL_STATIC_DRAW);
				glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
				glEnableVertexAttribArray(1);
				glBindBuffer(GL_ARRAY_BUFFER, 0);

				glBindBuffer(GL_ARRAY_BUFFER, conditionVBO);
				glBufferData(GL_ARRAY_BUFFER, conditionCnt * 1 * sizeof(GLfloat), conditionBuffer, GL_STATIC_DRAW);
				glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
				glEnableVertexAttribArray(2);
				glBindBuffer(GL_ARRAY_BUFFER, 0);

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshEBcnt * sizeof(GLuint), meshEB, GL_STATIC_DRAW);

			}
			glBindVertexArray(0);

			glBindVertexArray(meshVAO);
			glDrawElements(GL_TRIANGLES, meshEBcnt, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
		}
#endif
		{
			GLfloat * tempptr;
			GLuint tempuint;
			simulate->exportBendConditionData(tempptr, tempuint);
			
			// debug buffer
			GLfloat * fBarycentreBuffer = nullptr, *fNormalBuffer = nullptr;
			GLuint fSize;
			clothPiece->exportFaceNorm3fBuffer(fBarycentreBuffer, fNormalBuffer, fSize);

			Shader debugShader = ResourceManager::GetShader("cloth_piece_debug");
			debugShader.Use();
			glm::mat4 model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
			model = glm::translate(model, glm::vec3(0.0f, 0.40f, 0.0f)); // Translate it down a bit so it's at the center of the scene
			glUniformMatrix4fv(glGetUniformLocation(debugShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(debugShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(glGetUniformLocation(debugShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

			//glUniform3f(glGetUniformLocation(debugShader.Program, "viewPos"), camera.Position.x, camera.Position.y, camera.Position.z);
			//glUniform3f(glGetUniformLocation(debugShader.Program, "light.direction"), 0.0f, 1.0f, 0.0f);
			//glUniform3f(glGetUniformLocation(debugShader.Program, "light.color"), 0.3f, 0.49f, 0.85f);
			///* from http://devernay.free.fr/cours/opengl/materials.html */
			//glUniform3f(glGetUniformLocation(debugShader.Program, "material.ambient"), 0.19225f, 0.19225f, 0.19225f);
			//glUniform3f(glGetUniformLocation(debugShader.Program, "material.diffuse"), 0.50754f, 0.50754f, 0.50754f);
			//glUniform3f(glGetUniformLocation(debugShader.Program, "material.specular"), 0.508273f, 0.508273f, 0.508273f);
			//glUniform1f(glGetUniformLocation(debugShader.Program, "material.shininess"), 0.4f);

			glBindVertexArray(debugVAO);
			{
				glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
				glBufferData(GL_ARRAY_BUFFER, fSize * 3 * sizeof(GLfloat), fBarycentreBuffer, GL_STATIC_DRAW);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
				glEnableVertexAttribArray(0);
				glBindBuffer(GL_ARRAY_BUFFER, 0);

				glBindBuffer(GL_ARRAY_BUFFER, debugNormalVBO);
				glBufferData(GL_ARRAY_BUFFER, fSize * 3 * sizeof(GLfloat), fNormalBuffer, GL_STATIC_DRAW);
				glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
				glEnableVertexAttribArray(1);
				glBindBuffer(GL_ARRAY_BUFFER, 0);
			}
			glBindVertexArray(0);

			glBindVertexArray(debugVAO);
			glDrawArrays(GL_POINTS, 0, fSize);
			glBindVertexArray(0);
		
		}
		
		// Swap the buffers
		glfwSwapBuffers(window);
	}

	glfwTerminate();
	return 0;
}

void basic_info()
{
	std::cout << "INFO::ENV: OpenGL version " << glGetString(GL_VERSION) << std::endl;
	std::cout << "INFO::ENV: sizeof GLfloat " << sizeof GLfloat << std::endl;
	std::cout << "INFO::ENV: sizeof GLuint " << sizeof GLuint << std::endl;
	GLuint indices[] = {  // Note that we start from 0!
		0, 1, 3,  // First Triangle
		1, 2, 3   // Second Triangle
	};
	std::cout << "INFO::ENV: sizeof indices " << sizeof indices << std::endl;

}


//#pragma region "User input"
//
//// Moves/alters the camera positions based on user input
//void Do_Movement()
//{
//	// Camera controls
//	if (keys[GLFW_KEY_W])
//		camera.ProcessKeyboard(FORWARD, deltaTime);
//	if (keys[GLFW_KEY_S])
//		camera.ProcessKeyboard(BACKWARD, deltaTime);
//	if (keys[GLFW_KEY_A])
//		camera.ProcessKeyboard(LEFT, deltaTime);
//	if (keys[GLFW_KEY_D])
//		camera.ProcessKeyboard(RIGHT, deltaTime);
//}
//
//// Is called whenever a key is pressed/released via GLFW
//void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
//{
//	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
//		glfwSetWindowShouldClose(window, GL_TRUE);
//
//	if (action == GLFW_PRESS)
//		keys[key] = true;
//	else if (action == GLFW_RELEASE)
//		keys[key] = false;
//}
//
//void mouse_callback(GLFWwindow* window, double xpos, double ypos)
//{
//	if (firstMouse)
//	{
//		lastX = xpos;
//		lastY = ypos;
//		firstMouse = false;
//	}
//
//	GLfloat xoffset = xpos - lastX;
//	GLfloat yoffset = lastY - ypos;
//
//	lastX = xpos;
//	lastY = ypos;
//
//	camera.ProcessMouseMovement(xoffset, yoffset);
//}
//
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
//{
//	camera.ProcessMouseScroll(yoffset);
//}
//
//#pragma endregion