#ifndef PX_CG_GLFW_HPP
#define PX_CG_GLFW_HPP

#include <string>
#include <stdexcept>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace px {

class GLFWError;

namespace glfw {

void init();
void terminate();
void errorCallback(int code, const char *description);

}}

class px::GLFWError : public std::exception
{

public:
    GLFWError(const std::string &msg, const int code=0)
            : msg(msg), err_code(code)
    {}
    const char *what() const noexcept override
    {
        return msg.data();
    }
    inline int code() const
    {
        return err_code;
    }

protected:
    std::string msg;
    int err_code;
};


#endif
