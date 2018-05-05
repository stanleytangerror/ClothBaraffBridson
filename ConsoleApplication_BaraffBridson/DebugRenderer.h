#pragma once

#include <vector>
#include "OpenGLContext.h"
#include "Singleton.h"

class IRenderBufferProvider
{
public:
	virtual const GLfloat *	Buffer() const = 0;
	virtual const GLuint	BufferSize() const = 0;
};

class DebugRenderer : public Singleton<DebugRenderer>
{
	class RenderData
	{
	public:
		RenderData(const IRenderBufferProvider* provider);

		const IRenderBufferProvider*	mProvider;
		GLuint							mVAO;
		GLuint							mVBO;
	};

public:
	DebugRenderer() : mDebugShader(nullptr) {}

	void Init(class Camera * camera);
	void Push(const IRenderBufferProvider* provider);
	void Draw();

private:
	std::vector<RenderData>		mRenderDataList;
	class Shader*				mDebugShader;
	class Camera*				mCamera;

	GLuint		mVAO;
	GLuint		mVBO;
};