#include "shader/base_shader.hpp"

#include <glm/gtc/type_ptr.hpp>

using namespace px;

Shader::Shader(const char *vertex_shader, const char *frag_shader,
               const char *geo_shader,
               const char *tc_shader,
               const char *te_shader)
    : _pid(0)
{
    init(vertex_shader, frag_shader, geo_shader, tc_shader, te_shader);
}

Shader::~Shader()
{
    glDeleteProgram(_pid);
}

[[noreturn]]
void Shader::err(std::string const &msg)
{
    throw OpenGLError(msg);
}

void Shader::init(const char *vertex_shader,
                  const char *frag_shader,
                  const char *geo_shader,
                  const char *tc_shader,
                  const char *te_shader)
{
    if (_pid == 0)
        _pid = glCreateProgram();

    unsigned int vs, fs, gs = 0, tcs = 0, tes = 0;
    SHADER_COMPILE_HELPER(vs, VERTEX, _pid, vertex_shader, err)
    SHADER_COMPILE_HELPER(fs, FRAGMENT, _pid, frag_shader, err)
    if (geo_shader != nullptr)
        SHADER_COMPILE_HELPER(gs, GEOMETRY, _pid, geo_shader, err);
    if (tc_shader != nullptr)
        SHADER_COMPILE_HELPER(tcs, TESS_CONTROL, _pid, tc_shader, err);
    if (te_shader != nullptr)
        SHADER_COMPILE_HELPER(tes, TESS_EVALUATION, _pid, te_shader, err);

    glLinkProgram(_pid);
    OPENGL_ERROR_CHECK(_pid, Program, LINK, err)

    glDetachShader(_pid, vs);
    glDetachShader(_pid, fs);
    glDetachShader(_pid, gs);
    glDetachShader(_pid, tcs);
    glDetachShader(_pid, tes);
    glDeleteShader(vs);
    glDeleteShader(fs);
    glDeleteShader(gs);
    glDeleteShader(tcs);
    glDeleteShader(tes);

    glUseProgram(0);
}

void Shader::use()
{
    glUseProgram(_pid);
}

void Shader::set(GLint id, glm::mat4 const &val) const
{
    glUniformMatrix4fv(id, 1, GL_FALSE, glm::value_ptr(val));
}
void Shader::set(std::string const &name, glm::mat4 const &val) const
{
    glUniformMatrix4fv(glGetUniformLocation(_pid, name.c_str()),
                       1, GL_FALSE, glm::value_ptr(val));
}
void Shader::set(const char *name, glm::mat4 const &val) const
{
    glUniformMatrix4fv(glGetUniformLocation(_pid, name),
                       1, GL_FALSE, glm::value_ptr(val));
}

void Shader::set(GLint id, bool val) const
{
    glUniform1i(id, val);
}
void Shader::set(std::string const &name, bool val) const
{
    glUniform1i(glGetUniformLocation(_pid, name.c_str()), val);
}
void Shader::set(const char *name, bool val) const
{
    glUniform1i(glGetUniformLocation(_pid, name), val);
}

void Shader::set(GLint id, int val) const
{
    glUniform1i(id, val);
}
void Shader::set(std::string const &name, int val) const
{
    glUniform1i(glGetUniformLocation(_pid, name.c_str()), val);
}
void Shader::set(const char *name, int val) const
{
    glUniform1i(glGetUniformLocation(_pid, name), val);
}

void Shader::set(GLint id, float val) const
{
    glUniform1f(id, val);
}
void Shader::set(std::string const &name, float val) const
{
    glUniform1f(glGetUniformLocation(_pid, name.c_str()), val);
}
void Shader::set(const char *name, float val) const
{
    glUniform1f(glGetUniformLocation(_pid, name), val);
}

void Shader::set(GLint id, glm::vec3 const &val) const
{
    glUniform3fv(id, 1, glm::value_ptr(val));
}
void Shader::set(std::string const &name, glm::vec3 const &val) const
{
    glUniform3fv(glGetUniformLocation(_pid, name.c_str()),
                 1, glm::value_ptr(val));
}
void Shader::set(const char *name, glm::vec3 const &val) const
{
    glUniform3fv(glGetUniformLocation(_pid, name),
                 1, glm::value_ptr(val));
}

void Shader::set(GLint id, glm::vec4 const &val) const
{
    glUniform4fv(id, 1, glm::value_ptr(val));
}
void Shader::set(std::string const &name, glm::vec4 const &val) const
{
    glUniform4fv(glGetUniformLocation(_pid, name.c_str()),
                 1, glm::value_ptr(val));
}
void Shader::set(const char *name, glm::vec4 const &val) const
{
    glUniform4fv(glGetUniformLocation(_pid, name),
                 1, glm::value_ptr(val));
}

void Shader::bind(GLuint id, int val) const
{
    glUniformBlockBinding(_pid, id, val);
}

void Shader::bind(std::string const &name, int val) const
{
    glUniformBlockBinding(_pid, glGetUniformBlockIndex(_pid, name.c_str()), val);
}

void Shader::bind(const char *name, int val) const
{
    glUniformBlockBinding(_pid, glGetUniformBlockIndex(_pid, name), val);
}

void Shader::output(std::string const &out)
{
    glBindFragDataLocation(_pid, 0, "color");
}
