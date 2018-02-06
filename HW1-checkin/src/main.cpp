#include "glfw.hpp"
#include "app.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.hpp"


int main()
{
    px::glfw::init();

    auto app = px::App::instance();
    app->init(false);

    while(app->run());

    px::glfw::terminate();
    return 0;
}
