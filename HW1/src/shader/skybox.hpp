#ifndef PX_CG_SHADER_SKYBOX_HPP
#define PX_CG_SHADER_SKYBOX_HPP

#include "shader/base_shader.hpp"

namespace px
{
class SkyBox;
}

class px::SkyBox : public Shader
{
public:
    static const char *VS;
    static const char *FS;
    static const float SKY_VERTICES[3*6*6];

    SkyBox();
    SkyBox(std::string const &xp,
           std::string const &xn,
           std::string const &yp,
           std::string const &yn,
           std::string const &zp,
           std::string const &zn);
    ~SkyBox();

    void load(std::string const &xp,
              std::string const &xn,
              std::string const &yp,
              std::string const &yn,
              std::string const &zp,
              std::string const &zn);

    void render(glm::mat4 const &view,
                glm::mat4 const &proj);

protected:
    void init();

    unsigned int vao, vbo, texture;

};


#endif
