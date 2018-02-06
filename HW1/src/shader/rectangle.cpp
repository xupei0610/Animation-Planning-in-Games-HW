#include "shader/rectangle.hpp"
#include "glfw.hpp"
#include <glm/gtc/matrix_transform.hpp>

using namespace px;

const char *RectangleShader::VS =
#include "shader/glsl/orth_rectangle_with_gaussian.vs"
;

const char *RectangleShader::FS =
#include "shader/glsl/orth_rectangle_with_gaussian.fs"
;

RectangleShader::RectangleShader()
    : Shader(VS, FS), vao(0), vbo(0)
{
    output("color");

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void *)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
}

RectangleShader::~RectangleShader()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

float vertices[] = {
     // x     y       u     v
        0.0f, 0.0f,   0.0f, 1.0f,
        0.0f, 0.0f,   0.0f, 0.0f,
        0.0f, 0.0f,   1.0f, 0.0f,

        0.0f, 0.0f,   0.0f, 1.0f,
        0.0f, 0.0f,   1.0f, 0.0f,
        0.0f, 0.0f,   1.0f, 1.0f
};

void RectangleShader::render(float x, float y, float width, float height,
                             glm::vec4 const &color,
                             unsigned int texture_id)
{

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    activate();
    glBindVertexArray(vao);

    if (width > 2.0f || width < -2.0f)
    {
        int w, h;
        glfwGetFramebufferSize(glfwGetCurrentContext(), &w, &h);
        set("proj", glm::ortho(0.0f, static_cast<float>(w),
                               0.0f, static_cast<float>(h)));
    }
    else
        set("proj", glm::mat4());

    set("rect_color", color);

    set("use_tex", texture_id == 0 ? (glBindTexture(GL_TEXTURE_2D, 0), 0)
                                   : (glBindTexture(GL_TEXTURE_2D, texture_id), 1));

    vertices[0] = x;         vertices[1] = y + height;
    vertices[4] = x;         vertices[5] = y;
    vertices[8] = x + width; vertices[9] = y;

    vertices[12] = x;         vertices[13] = y + height;
    vertices[16] = x + width; vertices[17] = y;
    vertices[20] = x + width; vertices[21] = y + height;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    activate(false);
}