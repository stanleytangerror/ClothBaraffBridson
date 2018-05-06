#include "Simulator.h"
#include "Screen.h"
#include "Scene.h"
#include "ResourceManager.h"
#include "Config.h"
#include "EventManager.h"
#include "Clock.h"
#include "ContactHandler.h"
#include "DebugRenderer.h"

void Simulator::run()
{
	while (!Screen::closed())
	{
		Clock::Instance()->Tick(1.f);

		{
			EventManager::active(eventManager);
			Screen::pullEvents();
			eventManager->handleEvents();
			if (!Clock::Instance()->paused())
			{
				simulateInternal();
			}
		}
		Scene::renderScene();
		Screen::swapBuffers();
	}
}

void Simulator::init()
{
	Screen::initEnv();

	DebugRenderer::Instance()->Init(viewer->getCamera());

	//////////////////////////////////////////////////////////////////////////
	// load shaders
	ResourceManager::LoadShader("model_loading", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.frag");
	ResourceManager::LoadShader("background_cube", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.frag");
	ResourceManager::LoadShader("cloth_piece", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.frag");
	ResourceManager::LoadShader("rigid_body", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.frag");
	ResourceManager::LoadShader("cloth_piece_normal", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.gs");
	ResourceManager::LoadShader("bounding_box", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.gs");
	ResourceManager::LoadShader("contact_point", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\contact_point.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\contact_point.frag");
	ResourceManager::LoadShader("cloth", ".\\ConsoleApplication_BaraffBridson\\cloth_vs.glsl", ".\\ConsoleApplication_BaraffBridson\\cloth_frag.glsl");

	//////////////////////////////////////////////////////////////////////////
	// load models
	Model squarePlaneModel((GLchar *)Config::modelPath.c_str(), (aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	Model sphereModel((GLchar *)Config::spherePath.c_str(), (aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));

	//////////////////////////////////////////////////////////////////////////
	// create cloth
	{
		clothPiece = new SurfaceMeshObject(3);
		clothPiece->import(squarePlaneModel.getMeshes()[0]);
		{
			Eigen::Matrix4f affine;
			affine <<
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 2.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f;
			clothPiece->Affine(affine);
		}
		clothPiece->useVTexCoord2DAsVPlanarCoord3f();
		clothPiece->addPositionsProperty();

		Scene::add_component(new SceneClothPiece(
			ResourceManager::GetShader("cloth"),
			ResourceManager::GetShader("cloth_piece_normal"),
			clothPiece,
			viewer->getCamera()));
	}

	//////////////////////////////////////////////////////////////////////////
	// rigid body - sphere
	{
		SurfaceMeshObject * meshObj = new SurfaceMeshObject(3);
		mRigidBodies.push_back(meshObj);

		meshObj->import(sphereModel.getMeshes()[0]);
		{
			Eigen::Matrix4f affine;
			affine <<
				3.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 3.0f, 0.0f, -2.0f,
				0.0f, 0.0f, 3.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f;
			meshObj->Affine(affine);
		}
		meshObj->addPositionsProperty();

		Scene::add_component(new SceneRigidBody(
			ResourceManager::GetShader("cloth"),
			meshObj,
			viewer->getCamera()));
	}

	//////////////////////////////////////////////////////////////////////////
	// rigid body - plane
	{
		SurfaceMeshObject * meshObj = new SurfaceMeshObject(3);
		mRigidBodies.push_back(meshObj);

		meshObj->import(squarePlaneModel.getMeshes()[0]);
		{
			Eigen::Matrix4f affine;
			affine <<
				3.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 3.0f, 0.0f, -5.0f,
				0.0f, 0.0f, 3.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f;
			meshObj->Affine(affine);
		}
		meshObj->addPositionsProperty();

		Scene::add_component(new SceneRigidBody(
			ResourceManager::GetShader("cloth"),
			meshObj,
			viewer->getCamera()));
	}

	//////////////////////////////////////////////////////////////////////////
	// simulation
	clothDynamics = new BaraffDynamics(clothPiece, mRigidBodies);

	Scene::load();

	//////////////////////////////////////////////////////////////////////////
	// input system
	eventManager = new EventManager(Screen::window);
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->viewer->keyboard_press(keyMask); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {
		if (keyMask[GLFW_KEY_C])
		{
			Clock::Instance()->resume();
			Clock::Instance()->PushFrameCounter(new FrameCounter(40, []() { Clock::Instance()->pause(); }));
		}
	});
	eventManager->registerMouseEventHandler([this](GLfloat xpos, GLfloat ypos, GLfloat xlast, GLfloat ylast) -> void {this->viewer->move_mouse(xpos, ypos, xlast, ylast); });
	eventManager->registerScrollEventHandler([this](GLfloat scrollX, GLfloat scrollY) -> void {this->viewer->scroll_mouse(scrollX, scrollY); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->pauseEventHandle(keyMask); });
}

void Simulator::simulateInternal()
{
	clothDynamics->stepforward(0.02f);
	clothDynamics->writeBack();
	clothDynamics->RecomputeNormals();
}

void Simulator::pauseEventHandle(bool const * const keyMask)
{
	if (keyMask[GLFW_KEY_Z])
	{
		Clock::Instance()->pause();
	}
	if (keyMask[GLFW_KEY_X])
	{
		Clock::Instance()->resume();
	}
}
