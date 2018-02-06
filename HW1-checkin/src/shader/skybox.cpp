#include "shader/skybox.hpp"
#include "global.hpp"

#include "stb_image.hpp"

using namespace px;

const char *SkyBox::VS =
#include "shader/glsl/cube_skybox.vs"
;

const char *SkyBox::FS =
#include "shader/glsl/cube_skybox.fs"
;

const float SkyBox::SKY_VERTICES[] = {
//      x      y      z
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

SkyBox::SkyBox()
    : Shader(VS, FS), vao(0), vbo(0), texture(0)
{
    init();
}

SkyBox::SkyBox(std::string const &xp,
               std::string const &xn,
               std::string const &yp,
               std::string const &yn,
               std::string const &zp,
               std::string const &zn)
        : Shader(VS, FS), vao(0), vbo(0), texture(0)
{
    init();
    load(xp, xn, yp, yn, zp, zn);
}

SkyBox::~SkyBox()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteTextures(1, &texture);
}

void SkyBox::init()
{
    if (vao == 0)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenTextures(1, &texture);
    }

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(SKY_VERTICES), SKY_VERTICES, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
}

void SkyBox::load(std::string const &xp,
                  std::string const &xn,
                  std::string const &yp,
                  std::string const &yn,
                  std::string const &zp,
                  std::string const &zn)
{

    int width, height, ch;

    glBindTexture(GL_TEXTURE_CUBE_MAP, texture);

    auto ptr = stbi_load(xp.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    ptr = stbi_load(xn.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    ptr = stbi_load(yp.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    ptr = stbi_load(yn.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    ptr = stbi_load(zp.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    ptr = stbi_load(zn.c_str(), &width, &height, &ch, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, ptr);
    stbi_image_free(ptr);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    use();
    set("skybox", 0);
    bind("GlobalAttributes", 0);
}

void SkyBox::render()
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    use();
    glBindVertexArray(vao);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, texture);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glBindVertexArray(0);

    glDepthFunc(GL_LESS);
}

