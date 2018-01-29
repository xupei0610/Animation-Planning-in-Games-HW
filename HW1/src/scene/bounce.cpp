#include "scene/bounce.hpp"
#include "scene.hpp"
#include "item/light_ball.hpp"
#include "util/random.hpp"


using namespace px;

scene::Bounce::Bounce()
    : Empty()
{}

void scene::Bounce::restart(Scene &scene)
{
    Empty::restart(scene);

    auto t_x = width*2;
    auto t_z = height*2;

    for (decltype(t_x) x = 0; x < t_x; ++x)
    {
        for (decltype(t_z) z = 0; z < t_z; ++z)
        {
            auto obj = new item::LightBall(glm::vec3(x * 0.5, 7.5f + 2.5f * rnd() , z * 0.5f),
                                             glm::vec3(0.025f));
            obj->resistance(scene.opt->resistance());
            obj->acceleration(scene.opt->gravity());

            obj->color().r = rnd() * .5f + .5f;
            obj->color().g = rnd() * .5f + .5f;
            obj->color().b = rnd() * .5f + .5f;
            auto l = obj->light();
            l.diffuse = obj->color();
            l.ambient = l.diffuse * .05f;
            l.specular = l.diffuse * .5f;
            obj->enlight(l);

            scene.add(obj);
        }
    }
}
