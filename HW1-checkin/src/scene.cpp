#include "scene.hpp"
#include "app.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace px;

Scene::Scene(Option *opt)
        : opt(opt), cam(), character(&cam, this),
          geo_shader(nullptr), light_shader(nullptr),
          ubo(0), deferred_vao(0), deferred_vbo(0),
          deferred_fbo{0}, deferred_rbo{0}, deferred_out{0},
          _scene(nullptr)
{}

Scene::~Scene()
{
    glDeleteBuffers(1, &ubo);
    glDeleteVertexArrays(1, &deferred_vao);
    glDeleteBuffers(1, &deferred_vbo);
    glDeleteFramebuffers(2, deferred_fbo);
    glDeleteTextures(7, deferred_out);
    glDeleteRenderbuffers(2, deferred_rbo);

    delete geo_shader;
    delete light_shader;
}

[[noreturn]]
void Scene::err(std::string const &msg)
{
    throw AppError("App Error: " + msg);
}

std::shared_ptr<Item> const &Scene::add(Item *obj)
{
    objs.emplace_back(obj);
    objs.back()->init();

    return objs.back();
}

void Scene::load(scene::BaseScene *s)
{
    _scene = s;
    _scene->init(*this);
}

void Scene::init()
{
    if (geo_shader == nullptr)
    {
        geo_shader = new Shader(
        #include "shader/glsl/scene_geo_shader.vs"
        ,
        #include "shader/glsl/scene_geo_shader.fs"
        );
        light_shader = new Shader(
        #include "shader/glsl/scene_light_shader.vs"
        ,
        #include "shader/glsl/scene_light_shader.fs"
        );
        glGenBuffers(1, &ubo);
        glGenVertexArrays(1, &deferred_vao);
        glGenBuffers(1, &deferred_vbo);
        glGenFramebuffers(2, deferred_fbo);
        glGenTextures(7, deferred_out);
        glGenRenderbuffers(2, deferred_rbo);
    }

    geo_shader->use();
    geo_shader->set("material.displace", 0);
    geo_shader->set("material.normal",   1);
    geo_shader->set("material.specular", 2);
    geo_shader->set("material.displace", 3);
    glBindBuffer(GL_UNIFORM_BUFFER, ubo);
    glBufferData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4)+sizeof(glm::vec3), nullptr, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    geo_shader->bind("GlobalAttributes", 0);
    glBindBufferRange(GL_UNIFORM_BUFFER, 0, ubo, 0, 2*sizeof(glm::mat4)+sizeof(glm::vec3));


    light_shader->use();
    light_shader->set("gPosition", 0);
    light_shader->set("gColor",    1);
    light_shader->set("gView",     2);
    light_shader->set("gNormal",   3);
    light_shader->set("gDiffuse",  4);
    light_shader->set("gSpecular", 5);
    light_shader->output("color");
    constexpr static float vertices[] = {
            // x     y       u     v
            -1.0f,  1.0f,   0.0f, 1.0f,
            -1.0f, -1.0f,   0.0f, 0.0f,
             1.0f,  1.0f,   1.0f, 1.0f,
             1.0f, -1.0f,   1.0f, 0.0f,
    };
    glBindVertexArray(deferred_vao);
    glBindBuffer(GL_ARRAY_BUFFER, deferred_vbo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void *)(2*sizeof(float)));
    glEnableVertexAttribArray(1);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glClearColor(.2f, .3f, .3f, 1.f);
}

void Scene::restart()
{
    if (scene() == nullptr)
        err("No scene loaded.");

    objs.clear();

    _scene->restart(*this);
}

void Scene::upload()
{
    _scene->upload(*geo_shader);
}

void Scene::render()
{
//    glEnable(GL_CULL_FACE);
//    glCullFace(GL_BACK);
//    glFrontFace(GL_CCW);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS);

    glBindFramebuffer(GL_FRAMEBUFFER, deferred_fbo[0]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, deferred_out[1], 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    geo_shader->use();
    upload();

    glBindBuffer(GL_UNIFORM_BUFFER, ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(cam.viewMat()));
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(cam.projMat()));
    glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4), sizeof(glm::vec3), glm::value_ptr(cam.pos()));
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    geo_shader->set("model", Camera::IDENTITY_MAT4);
    geo_shader->set("headlight.pos", cam.pos());
    geo_shader->set("headlight.dir", cam.direction());

    // render normal objects, would be influenced by lighting
    _scene->render(*geo_shader);
    for (auto & o : objs)
    {
        if (o->preRender())
            o->render(*geo_shader);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // render lighting
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    light_shader->use();
    glBindVertexArray(deferred_vao);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, deferred_out[0]);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, deferred_out[2]);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, deferred_out[3]);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, deferred_out[4]);
    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_2D, deferred_out[5]);
    auto idx = 0;
    for (auto & o : objs)
    {
        if (o->lighting())
        {
            auto & light = o->light();
            light_shader->set("light[" + std::to_string(idx) + "].pos",      o->pos());
            light_shader->set("light[" + std::to_string(idx) + "].ambient",  light.ambient);
            light_shader->set("light[" + std::to_string(idx) + "].diffuse",  light.diffuse);
            light_shader->set("light[" + std::to_string(idx) + "].specular", light.specular);
            light_shader->set("light[" + std::to_string(idx) + "].coef_a0",  light.coef.x);
            light_shader->set("light[" + std::to_string(idx) + "].coef_a1",  light.coef.y);
            light_shader->set("light[" + std::to_string(idx) + "].coef_a2",  light.coef.z);
            ++idx;
            if (idx == 100)
            {
                glBindFramebuffer(GL_FRAMEBUFFER, deferred_fbo[1]);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT6, GL_TEXTURE_2D, deferred_out[6], 0);

                light_shader->set("tot_light", idx);
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, deferred_out[1]);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                std::swap(deferred_out[1], deferred_out[6]);
                idx = 0;
            }
        }
    }
    light_shader->use();
    light_shader->set("tot_light", idx);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, deferred_out[1]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);

    // render objects that would not be influenced by lighting
    glBindFramebuffer(GL_READ_FRAMEBUFFER, deferred_fbo[0]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, cam.width(), cam.height(), 0, 0, cam.width(), cam.height(), GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (auto & o : objs)
    {
        if (o->postRender())
            o->render();
    }
    _scene->render();
}

void Scene::resize(int w, int h)
{
    cam.width() = w;
    cam.height() = h;
    glViewport(0, 0, cam.width(), cam.height());
    cam.updateProj();

    if (deferred_fbo[0] != 0)
    {
        for (auto i = 0; i < 5; ++i)
        {
            glBindTexture(GL_TEXTURE_2D, deferred_out[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, cam.width(), cam.height(), 0, GL_RGB, GL_FLOAT, 0);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        }
        glBindTexture(GL_TEXTURE_2D, deferred_out[5]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, cam.width(), cam.height(), 0, GL_RGBA, GL_FLOAT, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, deferred_out[6]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, cam.width(), cam.height(), 0, GL_RGB, GL_FLOAT, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);

        // bind renderbuffer output
        glBindFramebuffer(GL_FRAMEBUFFER, deferred_fbo[0]);
        for (auto i = 0; i < 6; ++i)
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, deferred_out[i], 0);
        glBindRenderbuffer(GL_RENDERBUFFER, deferred_rbo[0]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, cam.width(), cam.height());
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, deferred_rbo[0]);
        GLuint attach[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2,
                            GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5};
        glDrawBuffers(6, attach);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            err("Failed to generate frame buffer");

        glBindFramebuffer(GL_FRAMEBUFFER, deferred_fbo[1]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT6, GL_TEXTURE_2D, deferred_out[6], 0);
        glBindRenderbuffer(GL_RENDERBUFFER, deferred_rbo[1]);
        attach[0] = GL_COLOR_ATTACHMENT6;
        glDrawBuffers(1, attach);
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            err("Failed to generate frame buffer");
    }
}
bool Scene::run(float dt)
{
    auto field_height = 0.f;
    glm::vec3 field_norm(0.f, 1.f, 0.f);

    auto movement = character.makeAction(dt);
    if (!character.canFloat() && !character.isAscending())
    {
        if (cam.pos().y - character.characterHeight() > field_height)
            movement.y -= character.dropSpeed() * dt;

        auto lowest = field_height + character.characterHeight();
        if (cam.pos().y + movement.y <= lowest)
        {
            movement.y = lowest - cam.pos().y;
            character.disableDropping();
        }
        else
            character.enableDropping();
    }
    cam.pos() += movement;

    cam.updateView();

//    auto tot = static_cast<int>(objs.size());
//#pragma omp parallel for num_threads(8)
//    for (auto i = 0; i < tot; ++i)
    for (auto it = objs.begin(); it != objs.end();)
    {
        (*it)->update(dt);
        if ((*it)->alive())
        {
            if ((*it)->move())
            {
                auto h = field_height + (*it)->hsize().y;
                auto above_field = (*it)->pos().y >= h;
                (*it)->move((*it)->movement());
                auto & pos = (*it)->pos();
                if (above_field && pos.y < h)
                {
                    glm::vec3 at(pos.x, h, pos.z);
                    (*it)->place(at);
                    (*it)->hit(at, field_norm);
                }
            }
            ++it;
        }
        else
        {
            it = objs.erase(it);
            continue;
        }
    }

    _scene->update(dt);

    return true;
}
