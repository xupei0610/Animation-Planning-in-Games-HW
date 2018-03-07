// #include <omp.h>

#include "glfw.hpp"
#include "app.hpp"
#include "config.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.hpp"

int main()
{
    // omp_set_dynamic(0);
    // omp_set_num_threads(BENCHMARK_OMP_THREADS);

    px::glfw::init();

    auto app = px::App::instance();
    app->init(
//#ifndef NFULLSCREEN
             true
//#else
//             false
//#endif
             );


    while(app->run());

    px::glfw::terminate();
    return 0;
}
