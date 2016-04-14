#include "OpenGLContext.h"
#include "Scene.h"
#include "Screen.h"
#include "Camera.h"
#include "ResourceManager.h"

std::vector<SceneComponent *> Scene::render_list;

Scene::Index Scene::add_component(SceneComponent * com)
{
	int index = Scene::render_list.size();
	Scene::render_list.push_back(com);
	return index;
}

void Scene::erase_component(Scene::Index index)
{
	assert(index >= 0 && index < Scene::render_list.size());
	auto iter = Scene::render_list.begin();
	for (int _i = 0; _i < index; ++_i, ++iter);
	Scene::render_list.erase(iter);
}

void Scene::draw()
{
	glClearColor(0.6f, 0.6f, 0.6f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (SceneComponent * com : Scene::render_list)
	{
		com->draw();
	}
	//glfwSwapBuffers(window);

}

void Scene::update()
{
	for (SceneComponent * com : Scene::render_list)
	{
		com->update();
	}
}

void Scene::load()
{
	for (SceneComponent * com : Scene::render_list)
	{
		com->load();
	}
}

/* ----------- SceneEnv ------------- */

void SceneEnv::draw()
const
{

	// draw cubemap
	glDepthMask(GL_FALSE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	//	Shader backgoroundShader = ResourceManager::GetShader("background_cube");
	shader->Use();

	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "view"), 1, GL_FALSE, glm::value_ptr(glm::mat4(glm::mat3(view))));

	glBindVertexArray(vao);
	{
		glActiveTexture(GL_TEXTURE0);
		glUniform1i(glGetUniformLocation(shader->Program, "background"), GL_TEXTURE0 - GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMap->ID);

		glDrawArrays(GL_TRIANGLES, 0, 36);
	}
	
	glBindVertexArray(0);

	
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDepthMask(GL_TRUE);

}

void SceneEnv::load()
{
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	{
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
	}
	glBindVertexArray(0);

}

void SceneEnv::update()
{
	projection = glm::perspective(camera->Zoom, Screen::aspectRatio, 0.1f, 100.0f);
	view = camera->GetViewMatrix();
}

/* ----------- SceneClothPiece ------------- */

void SceneClothPiece::draw()
const 
{
	glDepthMask(GL_TRUE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	/* ------- draw cloth piece ------- */
	clothPieceShader->Use();

	glUniformMatrix4fv(glGetUniformLocation(clothPieceShader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(clothPieceShader->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(clothPieceShader->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "viewPos"), viewPos.x, viewPos.y, viewPos.z);
	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "light.direction"), 0.0f, 1.0f, 0.0f);
	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "light.color"), 0.3f, 0.49f, 0.85f);
	/* from http://devernay.free.fr/cours/opengl/materials.html */
	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "material.ambient"), 0.19225f, 0.19225f, 0.19225f);
	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "material.diffuse"), 0.50754f, 0.50754f, 0.50754f);
	glUniform3f(glGetUniformLocation(clothPieceShader->Program, "material.specular"), 0.508273f, 0.508273f, 0.508273f);
	glUniform1f(glGetUniformLocation(clothPieceShader->Program, "material.shininess"), 0.4f);

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

		//glBindBuffer(GL_ARRAY_BUFFER, conditionVBO);
		//glBufferData(GL_ARRAY_BUFFER, conditionCnt * 1 * sizeof(GLfloat), conditionBuffer, GL_STATIC_DRAW);
		//glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
		//glEnableVertexAttribArray(2);
		//glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshEBcnt * sizeof(GLuint), meshEB, GL_STATIC_DRAW);
		glDrawElements(GL_TRIANGLES, meshEBcnt, GL_UNSIGNED_INT, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	}
	glBindVertexArray(0);
	
	//glGetUniformLocation(ResourceManager::GetShader("cloth_piece_normal")->Program, "projection");

	//glBindVertexArray(meshVAO);
	//glBindVertexArray(0);


	//clothPieceShader->Use();
	//glGetUniformLocation(clothPieceShader->Program, "projection");
	//glGetUniformLocation(9, "projection");

	//return;

	/* ------- draw cloth normal ------- */
	debugShader->Use();

	//auto p1 = ResourceManager::GetShader("cloth_piece_normal")->Program;
	//auto p2 = debugShader->Program;
	//std::cout << p1 + p2 << std::endl;
	////glGetUniformLocation(6, "projection");
	//glGetUniformLocation(debugShader->Program, "projection");
	//glGetUniformLocation(13, "projection");
	//glGetUniformLocation(9, "projection");
	////glGetUniformLocation(3, "projection");
	//auto loc = glGetUniformLocation(ResourceManager::GetShader("cloth_piece_normal")->Program, "projection");
	//loc = glGetUniformLocation(debugShader->Program, "projection");
	glUniformMatrix4fv(glGetUniformLocation(debugShader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(debugShader->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(debugShader->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

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

		//glBindVertexArray(0);

		//glBindVertexArray(debugVAO);
		glDrawArrays(GL_POINTS, 0, fSize);
	}
	glBindVertexArray(0);
	
}

void SceneClothPiece::update()
{
	projection = glm::perspective(camera->Zoom, Screen::aspectRatio, 0.1f, 100.0f);
	view = camera->GetViewMatrix();
	model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
	model = glm::translate(model, glm::vec3(0.0f, 0.40f, 0.0f)); // Translate it down a bit so it's at the center of the scene

	viewPos = camera->Position;

	
	meshVB = nullptr;
	meshVNormalB = nullptr;
	meshEB = nullptr;
	meshVBcnt = 0, meshEBcnt = 0;
	//conditionBuffer = nullptr;
	//conditionCnt = 0;
	clothPiece->exportPos3fNorm3fBuffer(meshVB, meshVNormalB, meshVBcnt, meshEB, meshEBcnt);

	fBarycentreBuffer = nullptr;
	fNormalBuffer = nullptr;
	fSize = 0;
	clothPiece->exportFaceNorm3fBuffer(fBarycentreBuffer, fNormalBuffer, fSize);
	
}

void SceneClothPiece::load()
{
	// cloth piece surface
	//clothPieceShader = ResourceManager::GetShader("cloth_piece");
	std::cout << "INFO::LOAD SCENE cloth piece scene" << std::endl;

	glGenVertexArrays(1, &meshVAO);
	std::cout << "mesh vao " << meshVAO << std::endl;
	glGenBuffers(1, &meshVBO);
	std::cout << "mesh vbo " << meshVBO << std::endl;
	glGenBuffers(1, &meshVNormalBO);
	std::cout << "mesh vbNormalo " << meshVNormalBO << std::endl;
	glGenBuffers(1, &meshEBO);
	std::cout << "mesh ebo " << meshEBO << std::endl;
	glGenBuffers(1, &conditionVBO);
	std::cout << "condition vbo " << conditionVBO << std::endl;


	// cloth piece normal
	//debugShader = ResourceManager::GetShader("cloth_piece_normal");

	glGenVertexArrays(1, &debugVAO);
	std::cout << "debug vao " << debugVAO << std::endl;
	glGenBuffers(1, &debugVBO);
	std::cout << "debug vbo " << debugVBO << std::endl;
	glGenBuffers(1, &debugNormalVBO);
	std::cout << "debug normal vbo " << debugNormalVBO << std::endl;

}

/* ----------- SceneRigidBody ------------- */

void SceneRigidBody::draw()
const
{
	glDepthMask(GL_TRUE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	/* ------- draw cloth piece ------- */
	shader->Use();

	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

	glUniform3f(glGetUniformLocation(shader->Program, "viewPos"), viewPos.x, viewPos.y, viewPos.z);
	glUniform3f(glGetUniformLocation(shader->Program, "light.direction"), 0.0f, 1.0f, 0.0f);
	glUniform3f(glGetUniformLocation(shader->Program, "light.color"), 0.45f, 0.45f, 0.45f);
	/* from http://devernay.free.fr/cours/opengl/materials.html */
	glUniform3f(glGetUniformLocation(shader->Program, "material.ambient"), 0.19225f, 0.19225f, 0.19225f);
	glUniform3f(glGetUniformLocation(shader->Program, "material.diffuse"), 0.50754f, 0.50754f, 0.50754f);
	glUniform3f(glGetUniformLocation(shader->Program, "material.specular"), 0.508273f, 0.508273f, 0.508273f);
	glUniform1f(glGetUniformLocation(shader->Program, "material.shininess"), 0.4f);

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

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshEBcnt * sizeof(GLuint), meshEB, GL_STATIC_DRAW);
		glDrawElements(GL_TRIANGLES, meshEBcnt, GL_UNSIGNED_INT, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	}
	glBindVertexArray(0);

}

void SceneRigidBody::update()
{
	projection = glm::perspective(camera->Zoom, Screen::aspectRatio, 0.1f, 100.0f);
	view = camera->GetViewMatrix();
	model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
	model = glm::translate(model, glm::vec3(0.0f, 0.40f, 0.0f)); // Translate it down a bit so it's at the center of the scene

	viewPos = camera->Position;

	meshVB = nullptr;
	meshVNormalB = nullptr;
	meshEB = nullptr;
	meshVBcnt = 0, meshEBcnt = 0;
	
	rigidBody->exportPos3fNorm3fBuffer(meshVB, meshVNormalB, meshVBcnt, meshEB, meshEBcnt);

}

void SceneRigidBody::load()
{
	std::cout << "INFO::LOAD SCENE rigid body scene" << std::endl;
	glGenVertexArrays(1, &meshVAO);
	std::cout << "mesh vao " << meshVAO << std::endl;
	glGenBuffers(1, &meshVBO);
	std::cout << "mesh vbo " << meshVBO << std::endl;
	glGenBuffers(1, &meshVNormalBO);
	std::cout << "mesh vbNormalo " << meshVNormalBO << std::endl;
	glGenBuffers(1, &meshEBO);
	std::cout << "mesh ebo " << meshEBO << std::endl;

}

/* ----------- SceneAABBTree ------------- */

void SceneAABBox::draw()
const
{
	if (tree == nullptr)
		return;

	glDepthMask(GL_TRUE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	
	shader->Use();

	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(shader->Program, "model"), 1, GL_FALSE, glm::value_ptr(model));

	glBindVertexArray(vao);
	{
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, pointSize * 3 * sizeof(GLfloat), boxVerticesBuffer, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glDrawArrays(GL_LINES, 0, pointSize);
	}
	glBindVertexArray(0);

}

void SceneAABBox::load()
{
	//shader = ResourceManager::GetShader("bounding_box");
	std::cout << "INFO::LOAD SCENE AABBox scene" << std::endl;

	glGenVertexArrays(1, &vao);
	std::cout << "aabbox vao " << vao << std::endl;
	glGenBuffers(1, &vbo);
	std::cout << "aabbox vbo " << vbo << std::endl;

}

void SceneAABBox::update()
{
	if (tree == nullptr)
		return;

	projection = glm::perspective(camera->Zoom, Screen::aspectRatio, 0.1f, 100.0f);
	view = camera->GetViewMatrix();
	model = glm::scale(glm::mat4(), glm::vec3(0.20f, 0.20f, 0.20f));
	model = glm::translate(model, glm::vec3(0.0f, 0.40f, 0.0f)); // Translate it down a bit so it's at the center of the scene

	tree->exportAABBoxPositions(boxVerticesBuffer, pointSize);
}