#include "Simulator.h"
#include "Screen.h"
#include "Scene.h"
#include "ResourceManager.h"
#include "Config.h"
#include "EventManager.h"

void Simulator::run()
{
	while (!Screen::closed())
	{
		{
			EventManager::active(eventManager);
			Screen::pullEvents();
			eventManager->handleEvents();
			if (!clock->paused())
			{
				updateData();
			}
		}
		Scene::renderScene();
		Screen::swapBuffers();
	}
}

void Simulator::init()
{
	Screen::initEnv();
	loopCount = 0u;

	// ---------- Setup and compile our shaders -------------------
	ResourceManager::LoadShader("model_loading", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.frag");
	ResourceManager::LoadShader("background_cube", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.frag");
	ResourceManager::LoadShader("cloth_piece", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.frag");
	ResourceManager::LoadShader("rigid_body", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.frag");
	ResourceManager::LoadShader("cloth_piece_normal", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.gs");
	ResourceManager::LoadShader("bounding_box", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.gs");
	//ResourceManager::LoadShader("model_loading", ".\\model_loading.vs", ".\\model_loading.frag");
	//ResourceManager::LoadShader("background_cube", ".\\background_cube.vs", ".\\background_cube.frag");

	// ----------- load cube map ----------------
	std::vector<const GLchar*> faces;
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\top.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\bottom.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	ResourceManager::LoadCubeMap("background_texture", faces);

	// ----------- Load models ----------------
	Model ourModel((GLchar *)Config::modelPath.c_str(), 
		(aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	
	clothPiece = new ClothPiece(3);
	clothPiece->import(ourModel.getMeshes()[0]);
	clothPiece->useVTexCoord2DAsVPlanarCoord3f();
	
	// initial planar coordinates
	clothDynamics = new BaraffDynamics(clothPiece);

	auto env = new SceneEnv(ResourceManager::GetShader("background_cube"),
		ResourceManager::GetCubeMap("background_texture"), viewer->getCamera());
	Scene::add_component(env);
	auto clo = new SceneClothPiece(ResourceManager::GetShader("cloth_piece"),
		ResourceManager::GetShader("cloth_piece_normal"),
		clothPiece, viewer->getCamera());
	Scene::add_component(clo);
	clothPieceBoxSceneIndex = Scene::add_component(new SceneAABBox(ResourceManager::GetShader("bounding_box"),
		viewer->getCamera()));

	Model rigidBodyModel((GLchar *)Config::spherePath.c_str(),
		(aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	rigidBody = new ClothPiece(3);
	rigidBody->import(rigidBodyModel.getMeshes()[0]);
	auto sph = new SceneRigidBody(ResourceManager::GetShader("rigid_body"),
		rigidBody, viewer->getCamera());
	Scene::add_component(sph);
	rigidBodyBoxSceneIndex = Scene::add_component(new SceneAABBox(ResourceManager::GetShader("bounding_box"),
		viewer->getCamera()));

	Scene::load();

	// ------------ register event handlers ------------
	eventManager = new EventManager(Screen::window);
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->viewer->keyboard_press(keyMask); });
	eventManager->registerMouseEventHandler([this](GLfloat xpos, GLfloat ypos, GLfloat xlast, GLfloat ylast) -> void {this->viewer->move_mouse(xpos, ypos, xlast, ylast); });
	eventManager->registerScrollEventHandler([this](GLfloat scrollX, GLfloat scrollY) -> void {this->viewer->scroll_mouse(scrollX, scrollY); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->pauseEventHandle(keyMask); });

}

void Simulator::updateData()
{
	
#define USE_SIMULATION

#ifdef SHOW_MODEL
			Shader modelShader = ResourceManager::GetShader("model_loading");
			modelShader.Use();   // <-- Don't forget this one!
								 // Transformation matrices
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
			glm::mat4 model;
			model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
			glUniformMatrix4fv(glGetUniformLocation(modelShader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));
			ourModel.Draw(modelShader);
#endif

#ifdef USE_SIMULATION
		// Draw the loaded model
		//if (loop_cnt < init_loop)
		//{
			clothDynamics->stepforward(0.02f);
		//	loop_cnt += 1;
		//	std::cout << "simulation loop " << loop_cnt << std::endl;
		//}
		//if (keys[81])
		//{
		//	loop_cnt -= 1;
		//}
		clothDynamics->writeBack();
			
#endif

		std::cout << "create temp tree" << std::endl;
		auto tempTree = new TriangleTree(clothPiece->getMesh());
		(dynamic_cast<SceneAABBox *>(Scene::get_component(clothPieceBoxSceneIndex)))
			->setTree(tempTree);
		tempTree = new TriangleTree(rigidBody->getMesh());
		(dynamic_cast<SceneAABBox *>(Scene::get_component(rigidBodyBoxSceneIndex)))
			->setTree(tempTree);
}

void Simulator::pauseEventHandle(bool const * const keyMask)
{
	if (keyMask[GLFW_KEY_Z])
	{
		clock->pause();
	}
	if (keyMask[GLFW_KEY_X])
	{
		clock->resume();
	}
}



