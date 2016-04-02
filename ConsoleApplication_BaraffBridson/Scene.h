#ifndef SCENE_H
#define SCENE_H

#include "Shader.h"
#include "texture.h"
#include "Camera.h"
#include "ClothPiece.h"
#include "OtaduyContact.h"

#include <vector>

class SceneComponent;

class Scene
{
public:
	typedef int Index;

	static Index add_component(SceneComponent * com);

	static void erase_component(Index index);
	
	static SceneComponent * get_component(Index index)
	{
		assert(index >= 0 && index <= render_list.size());
		return render_list[index];
	}

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

	SceneEnv(Shader * shader, CubeMap const * cubeMap, Camera * camera) :
		SceneComponent(), cubeMap(cubeMap), shader(shader), camera(camera)
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

	SceneClothPiece(Shader * clothPieceShader, Shader * debugShader, ClothPiece const * const clothPiece, Camera * camera) :
		SceneComponent(), clothPiece(clothPiece), camera(camera),
		clothPieceShader(clothPieceShader), debugShader(debugShader)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneClothPiece() {}

protected:
	ClothPiece const * const clothPiece;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;
	glm::vec3 viewPos;

	/* cloth piece surface */
	Shader  * clothPieceShader = nullptr;
	Shader  * debugShader = nullptr;
	
	GLfloat * meshVB = nullptr;
	GLfloat * meshVNormalB = nullptr;
	GLuint * meshEB = nullptr;
	GLuint meshVBcnt = 0, meshEBcnt = 0;

	GLfloat * conditionBuffer = nullptr;
	GLuint conditionCnt = 0;

	GLuint meshVAO, meshVBO, meshVNormalBO, meshEBO, conditionVBO;

	/* cloth piece normal */
	GLuint debugVAO, debugVBO, debugNormalVBO;
	GLfloat * fBarycentreBuffer = nullptr;
	GLfloat * fNormalBuffer = nullptr;
	GLuint fSize;

	//GLfloat * tempptr;
	//GLuint tempuint;
	//stepforward->exportBendConditionData(tempptr, tempuint);



};

class TriangleTree;

class SceneAABBox : public SceneComponent
{
public:

	SceneAABBox(Shader * shader, Camera * camera) :
		SceneComponent(), camera(camera),
		shader(shader), tree(nullptr)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneAABBox() {}

	void setTree(TriangleTree * tree)
	{
		this->tree = tree;
	}

private:

	TriangleTree * tree;

	Shader * shader;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;

	GLuint vao, vbo;
	GLfloat * boxVerticesBuffer;
	//[6] = {
	//	0.2f, 0.2f, 0.2f, 
	//	0.6f, 0.6f, 0.6f };
	//GLfloat * boxVerticesBuffer;
	GLuint pointSize;

};
#endif
