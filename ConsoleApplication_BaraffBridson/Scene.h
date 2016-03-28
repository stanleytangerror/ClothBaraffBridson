#ifndef SCENE_H
#define SCENE_H

#include "Shader.h"
#include "texture.h"
#include "Camera.h"
#include "ClothPiece.h"

#include <vector>

class SceneComponent;

class Scene
{
public:
	static void add_component(SceneComponent * com);

	static void load();

	static void renderScene()
	{
		update();
		draw();
	}

private:
	explicit Scene() {}

	static std::vector<SceneComponent *> render_list;

	static void draw();

	static void update();

};

class SceneComponent
{
public:
	virtual void draw() const = 0;

	virtual void load() = 0;

	virtual void update() = 0;

	virtual ~SceneComponent() {}

};


class SceneEnv : public SceneComponent
{
public:

	SceneEnv(CubeMap const * cubeMap, Camera * camera) :
		SceneComponent(), cubeMap(cubeMap), shader(nullptr), camera(camera)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneEnv() {}

protected:

	CubeMap const * cubeMap;
	Shader  * shader;
	Camera * camera;

	glm::mat4 projection;
	glm::mat4 view;
	GLuint vao, vbo;

	GLfloat const cubeVertices[108] = {
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
};

class SceneClothPiece : public SceneComponent
{
public:

	SceneClothPiece(ClothPiece const * const clothPiece, Camera * camera) :
		SceneComponent(), clothPiece(clothPiece), camera(camera),
		clothPieceShader(nullptr), debugShader(nullptr)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneClothPiece() {}

protected:
	ClothPiece const * const clothPiece;

	/* cloth piece surface */
	Shader  * clothPieceShader = nullptr;
	GLfloat * meshVB = nullptr;
	GLfloat * meshVNormalB = nullptr;
	GLuint * meshEB = nullptr;
	GLuint meshVBcnt = 0, meshEBcnt = 0;

	GLfloat * conditionBuffer = nullptr;
	GLuint conditionCnt = 0;

	GLuint meshVAO, meshVBO, meshVNormalBO, meshEBO, conditionVBO;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;
	glm::vec3 viewPos;

	/* cloth piece normal */
	//GLfloat * tempptr;
	//GLuint tempuint;
	//stepforward->exportBendConditionData(tempptr, tempuint);

	GLuint debugVAO, debugVBO, debugNormalVBO;
	GLfloat * fBarycentreBuffer = nullptr, *fNormalBuffer = nullptr;
	GLuint fSize;

	Shader  * debugShader;

};

#endif
