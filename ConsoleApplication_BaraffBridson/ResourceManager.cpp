/*******************************************************************
** This code is part of Breakout.
**
** Breakout is free software: you can redistribute it and/or modify
** it under the terms of the CC BY 4.0 license as published by
** Creative Commons, either version 4 of the License, or (at your
** option) any later version.
******************************************************************/
#include "resourcemanager.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <SOIL.h>

// Instantiate static physics
std::map<std::string, Texture2D * >    ResourceManager::Textures;
std::map<std::string, Shader * >       ResourceManager::Shaders;
std::map<std::string, CubeMap * >		ResourceManager::CubeMaps;

Shader  *  ResourceManager::LoadShader(std::string name, const GLchar *vShaderFile, const GLchar *fShaderFile, const GLchar *gShaderFile)
{
	std::cout << "INFO::LOAD SHADER: " << name.c_str() << std::endl;
	Shaders[name] = loadShaderFromFile(vShaderFile, fShaderFile, gShaderFile);
	return Shaders[name];
}

Shader  *  ResourceManager::GetShader(std::string name)
{
	return Shaders.at(name);
}

Texture2D  *  ResourceManager::LoadTexture(const GLchar *file, GLboolean alpha, std::string name)
{
	Textures[name] = loadTextureFromFile(file, alpha);
	return Textures[name];
}

Texture2D  *  ResourceManager::GetTexture(std::string name)
{
	return Textures.at(name);
}

CubeMap  *  ResourceManager::LoadCubeMap(std::string name, std::vector<const GLchar *> faces)
{
	CubeMaps[name] = loadCubeMapFromFile(faces);
	return CubeMaps[name];
}

CubeMap  *  ResourceManager::GetCubeMap(std::string name)
{
	return CubeMaps.at(name);
}


void ResourceManager::Clear()
{
	// (Properly) delete all shaders	
	for (auto iter : Shaders)
		glDeleteProgram(iter.second->Program);
	// (Properly) delete all textures
	for (auto iter : Textures)
		glDeleteTextures(1, &iter.second->ID);
}

//#define DEBUG_SHADER

Shader * ResourceManager::loadShaderFromFile(const GLchar *vShaderFile, const GLchar *fShaderFile, const GLchar *gShaderFile)
{
	// 1. Retrieve the vertex/fragment source code from filePath
	std::string vertexCode;
	std::string fragmentCode;
	std::string geometryCode;
	try
	{
		// Open files
		std::ifstream vertexShaderFile(vShaderFile);
		std::ifstream fragmentShaderFile(fShaderFile);
		std::stringstream vShaderStream, fShaderStream;
		// Read file's buffer contents into streams
		vShaderStream << vertexShaderFile.rdbuf();
		fShaderStream << fragmentShaderFile.rdbuf();
		// close file handlers
		vertexShaderFile.close();
		fragmentShaderFile.close();
		// Convert stream into string
		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();
		// If geometry shader path is present, also load a geometry shader
		if (gShaderFile != nullptr)
		{
			std::ifstream geometryShaderFile(gShaderFile);
			std::stringstream gShaderStream;
			gShaderStream << geometryShaderFile.rdbuf();
			geometryShaderFile.close();
			geometryCode = gShaderStream.str();
		}
	}
	catch (std::exception e)
	{
		std::cout << "ERROR::SHADER: Failed to read shader files" << std::endl;
	}
#ifdef DEBUG_SHADER
	std::cout << "Vertex src " << std::endl << vertexCode << std::endl;
	std::cout << "Fragment src " << std::endl << fragmentCode << std::endl;
	if (gShaderFile != nullptr) std::cout << "Geometry src " << std::endl << geometryCode << std::endl;
#endif
	const GLchar *vShaderCode = vertexCode.c_str();
	const GLchar *fShaderCode = fragmentCode.c_str();
	const GLchar *gShaderCode = geometryCode.c_str();
	// 2. Now create shader object from source code
	Shader * shader = new Shader();
	shader->Compile(vShaderCode, fShaderCode, gShaderFile != nullptr ? gShaderCode : nullptr);
	return shader;
}

Texture2D *  ResourceManager::loadTextureFromFile(const GLchar *file, GLboolean alpha)
{
	// Create Texture object
	Texture2D * texture = new Texture2D();
	if (alpha)
	{
		texture->Internal_Format = GL_RGBA;
		texture->Image_Format = GL_RGBA;
	}
	// Load image
	int width, height;
	unsigned char* image = SOIL_load_image(file, &width, &height, 0, texture->Image_Format == GL_RGBA ? SOIL_LOAD_RGBA : SOIL_LOAD_RGB);
	// Now generate texture
	texture->Generate(width, height, image);
	// And finally free image data
	SOIL_free_image_data(image);
	return texture;
}

CubeMap *  ResourceManager::loadCubeMapFromFile(std::vector<const GLchar *> faces)
{
	CubeMap * cubeMap = new CubeMap();
	int width, height;
	std::vector<unsigned char *> faceImages;
	for (auto iter : faces)
		faceImages.push_back(SOIL_load_image(iter, &width, &height, 0, SOIL_LOAD_RGB));
	cubeMap->Generate(width, height, faceImages);
	for (auto iter : faceImages)
		SOIL_free_image_data(iter);
	return cubeMap;
}
