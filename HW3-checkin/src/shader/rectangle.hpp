#ifndef PX_CG_SHADER_RECTANGLE_HPP
#define PX_CG_SHADER_RECTANGLE_HPP

#include "shader/base_shader.hpp"

namespace px
{
class RectangleShader;
}

class px::RectangleShader : public Shader
{
public:
    static const char *VS;
    static const char *FS;

public:
    RectangleShader();
    ~RectangleShader();

    void render(float x, float y, float width, float height,
                glm::vec4 const &color,
                unsigned int texture_id);
protected:
    unsigned int vao, vbo;
};
#endif
