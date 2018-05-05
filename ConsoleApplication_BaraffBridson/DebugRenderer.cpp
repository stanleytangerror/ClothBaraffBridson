#include "DebugRenderer.h"
#include "OpenGLContext.h"
#include "Shader.h"
#include "Camera.h"
#include "Screen.h"
#include "ResourceManager.h"

DebugRenderer::RenderData::RenderData(const IRenderBufferProvider* provider)
	: mProvider(provider)
{
	glGenVertexArrays(1, &mVAO);
	glGenBuffers(1, &mVBO);
}

void DebugRenderer::Init(Camera * camera)
{
	ResourceManager::LoadShader("debug_render", 
		".\\ConsoleApplication_BaraffBridson\\debug_render.vs", 
		".\\ConsoleApplication_BaraffBridson\\debug_render.frag");
	mDebugShader = ResourceManager::GetShader("debug_render");
	
	mCamera = camera;
}

void DebugRenderer::Push(const IRenderBufferProvider* provider)
{
	mRenderDataList.push_back(provider);
}

#pragma optimize("", off)

void DebugRenderer::Draw()
{
	// update parameters
	glm::mat4 projection = glm::perspective(mCamera->Zoom, Screen::aspectRatio, 0.1f, 100.0f);
	glm::mat4 view = mCamera->GetViewMatrix();
	glm::vec3 viewPos = mCamera->Position;

	// start draw
	glDepthMask(GL_TRUE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	for (const auto & renderData : mRenderDataList)
	{
		if (!renderData.mProvider->BufferSize()) continue;

		mDebugShader->Use();

		glUniformMatrix4fv(glGetUniformLocation(mDebugShader->Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(mDebugShader->Program, "view"), 1, GL_FALSE, glm::value_ptr(view));
		//glUniform3f(glGetUniformLocation(mDebugShader->Program, "viewPos"), viewPos.x, viewPos.y, viewPos.z);

		glBindVertexArray(renderData.mVAO);
		{
			glBindBuffer(GL_ARRAY_BUFFER, renderData.mVBO);
			glBufferData(GL_ARRAY_BUFFER, renderData.mProvider->BufferSize() * sizeof(GLfloat), renderData.mProvider->Buffer(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0 * sizeof(GLfloat), (GLvoid *)0);
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glDrawArrays(GL_TRIANGLES, 0, renderData.mProvider->BufferSize() / 3);
		}
		glBindVertexArray(0);
	}

}
