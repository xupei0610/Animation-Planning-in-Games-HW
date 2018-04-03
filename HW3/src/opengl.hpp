#ifndef PX_CG_OPENGL_HPP
#define PX_CG_OPENGL_HPP

#include <stdexcept>

namespace px
{
class OpenGLError;
}

class px::OpenGLError : public std::exception
{

public:
    OpenGLError(const std::string &msg, const int code=0)
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
