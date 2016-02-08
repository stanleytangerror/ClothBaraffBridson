#ifndef MODEL_H
#define MODEL_H

#include <GL/glew.h>
#include <glm\glm.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Shader.h"
#include "Mesh.h"

#include <string>
#include <vector>

/* WARNING: can load any models with any edges (3, 4, ...),
	but can only draw models with triangles properly, 
	for not mention edges in mesh.indices (act like EBO)
	*/
class Model
{
public:
	/*  成员函数   */
	Model(const GLchar* path, aiPostProcessSteps options)
	{
		this->loadModel(path, options);
	}
	void Draw(Shader shader);

	const std::vector<Mesh> getMeshes() const {	return meshes;	}
private:
	std::vector<Mesh> meshes;
	std::string directory;
	std::vector<Texture> textures_loaded;	// Stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.

	/*  私有成员函数   */
	void loadModel(std::string path, aiPostProcessSteps options);
	void processNode(aiNode* node, const aiScene* scene);
	Mesh processMesh(aiMesh* mesh, const aiScene* scene);
	std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);
};

#endif