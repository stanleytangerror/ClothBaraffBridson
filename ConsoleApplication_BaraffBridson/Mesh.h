#ifndef MESH_H
#define MESH_H

#include "OpenGLContext.h"
#include "Shader.h"

#include <assimp\types.h>

#include <string>
#include <vector>

struct Vertex {
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec2 TexCoords;
};

struct Texture {
	GLuint id;
	std::string type;
	aiString path;
};

class Mesh {
public:
	/*  Mesh Data  */
	/* used as VBO */
	std::vector<Vertex> vertices;
	/* used as EBO */
	std::vector<GLuint> indices;
	std::vector<Texture> textures;
	/*  Functions  */
	Mesh(std::vector<Vertex> vertices, std::vector<GLuint> indices, std::vector<Texture> textures);
	void Draw(Shader shader);
private:
	/*  Render data  */
	GLuint VAO, VBO, EBO;
	/*  Functions    */
	void setupMesh();
};

#endif