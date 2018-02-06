#include "scene/vertex_particle_scene.hpp"
#include "scene.hpp"
#include "item/light_ball.hpp"
#include "util/random.hpp"
#include "app.hpp"

using namespace px;

scene::VertexParticleScene::VertexParticleScene()
    : BaseScene(), max_particles(50000), text(nullptr)
{}

void scene::VertexParticleScene::init(Scene &scene)
{
    if (text == nullptr)
    {
        text = new TextShader;
        text->setFontHeight(static_cast<std::size_t>(40));
        static const unsigned char TITLE_FONT_DATA[] = {
#include "font/Just_My_Type.dat"
        };
        text->addFont(TITLE_FONT_DATA, sizeof(TITLE_FONT_DATA));
    }

    glClearColor(0.f, 0.f, 0.f, 1.f);
}

void scene::VertexParticleScene::restart(Scene &scene)
{
//    auto t_x = static_cast<int>(std::sqrt(static_cast<float>(max_particles)));
//    auto t_z = max_particles / t_x;
//
//    max_particles = t_x * t_z;
//    _count = max_particles;

//    static Light light{glm::vec3(.5f), glm::vec3(.5f), glm::vec3(.5f), glm::vec3(1.f, 1.f, 2.f)};

//    for (decltype(t_x) x = 0; x < t_x; ++x)
//    {
//        for (decltype(t_z) z = 0; z < t_z; ++z)
//        {
//            auto obj = new item::LightBall(glm::vec3(0.25f + x * 0.25f, 7.5f + 2.5f * rnd() , 0.25f + z * 0.25f),
//                                             glm::vec3(0.025f));
//            obj->resistance(scene.opt->resistance());
//            obj->acceleration(scene.opt->gravity());
//
//            obj->color().r = rnd() * .5f + .5f;
//            obj->color().g = rnd() * .5f + .5f;
//            obj->color().b = rnd() * .5f + .5f;
//
////            light.diffuse = obj->color();
////            light.ambient = light.diffuse * .05f;
////            light.specular = light.diffuse * .5f;
////            obj->enlight(&light);
//
//            scene.add(obj);
//        }
//    }

    std::vector<float>(max_particles).swap(life);
    std::vector<item::LightBall *>(max_particles).swap(particles);
    _count = max_particles;
    for (decltype(max_particles) i = 0; i < max_particles; ++i)
    {
        auto theta = static_cast<float>(M_PI) * rnd();
        auto phi = static_cast<float>(M_PI) * (rnd() + 1) * .5f;
        auto r0 = std::cbrt((rnd() + 1) *.5f * 1000.f);
        glm::vec3 position;
        position.x = r0 * std::sin(phi) * std::cos(theta) + 20.f;
        position.z = r0 * std::sin(phi) * std::sin(theta) + 20.f;
        position.y = r0 * std::cos(phi);

        auto obj = new item::LightBall(position, glm::vec3(0.01f));
        obj->color().r = rnd() * .5f + .5f;
        obj->color().g = rnd() * .5f + .5f;
        obj->color().b = rnd() * .5f + .5f;
        obj->color().a = rnd() * .5f + .5f;
        obj->blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        scene.add(obj);
        life[i] = 2.f + rnd();
        particles[i] = obj;
    }

    scene.character.setShootable(false);
    scene.character.reset(0.f, scene.character.characterHeight(), 0.f, 135.f, 0.f);
}

void scene::VertexParticleScene::update(float dt)
{
    for (decltype(_count) i = 0; i < _count; ++i)
    {
        life[i] -= dt;
        if (life[i] <= 0.f)
        {
            glm::vec3 position;
            auto theta = static_cast<float>(M_PI) * rnd_np();
            auto phi =  std::acos(-1 + 2 * rnd());
            auto r0 = std::cbrt(rnd() * 1000.f);
            position.x = r0 * std::sin(phi) * std::cos(theta) + 20.f;
            position.z = r0 * std::sin(phi) * std::sin(theta) + 20.f;
            position.y = r0 * std::cos(phi);

            particles[i]->place(position);
            particles[i]->color().r = rnd() * .5f + .5f;
            particles[i]->color().g = rnd() * .5f + .5f;
            particles[i]->color().b = rnd() * .5f + .5f;
            particles[i]->color().a = rnd() * .5f + .5f;
            life[i] = 2.f + rnd();
        }
        else
            particles[i]->color().a *= life[i] / (life[i] + dt);
    }
    processInput(dt);
}

void scene::VertexParticleScene::render()
{
    text->render("Particles: " + std::to_string(count()),
                 10, 10, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);
    text->render("Render Mode: vertex array",
                 10, 30, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);

    auto h = App::instance()->frameHeight() - 25;
    if (max_particles != count())
    {
        text->render("Max Particles set to " + std::to_string(max_particles),
                     10, h - 20, .4f,
                     glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                     Anchor::LeftTop);
        text->render("Press R to reset the scene",
                     10, h, .4f,
                     glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                     Anchor::LeftTop);
    }
    text->render("Press Z to increase particles; Press X to decrease particles",
                 App::instance()->frameWidth() - 10, h, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::RightTop);
}

void scene::VertexParticleScene::processInput(float dt)
{
    static auto sum_dt = 0.f;
    static auto last_key = GLFW_KEY_UNKNOWN;
    static auto key_count = 0;

#define HOLD_KEY(Key)                                       \
    (last_key == Key && sum_dt > 0.01f && key_count == 10)

    auto & window = App::instance()->window();
    if (glfwGetKey(window, GLFW_KEY_Z) != GLFW_RELEASE)
    {
        if (last_key != GLFW_KEY_Z || sum_dt > 0.1f || HOLD_KEY(GLFW_KEY_Z))
        {
            max_particles = max_particles/500 * 500 + 500;
            sum_dt = 0;

            if (key_count < 10) ++key_count;
        }
        else
            sum_dt += dt;

        if (last_key != GLFW_KEY_Z)
        {
            last_key = GLFW_KEY_Z;
            key_count = 0;
        }

    }
    else if (glfwGetKey(window, GLFW_KEY_X) != GLFW_RELEASE)
    {
        if (max_particles > 500)
        {
            if (last_key != GLFW_KEY_X || sum_dt > 0.1f || HOLD_KEY(GLFW_KEY_X))
            {
                max_particles -= 1000;
                sum_dt = 0;

                if (key_count < 10) ++key_count;
            }
            else
                sum_dt += dt;
        }

        if (last_key != GLFW_KEY_X)
        {
            last_key = GLFW_KEY_X;
            key_count = 0;
        }
    }
    else
        last_key = GLFW_KEY_UNKNOWN;

#undef HOLD_KEY
}



