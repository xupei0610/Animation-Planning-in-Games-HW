#include "scene/empty_scene.hpp"
#include "scene.hpp"
#include "global.hpp"
#include "app.hpp"

#include "stb_image.hpp"

using namespace px;

scene::EmptyScene::EmptyScene()
        : BaseScene(),
          gravity(0.f, -5.f, 0.f), resistance(.5f), width(50), height(50),
          vao{0}, vbo{0}, texture{0},
          floor_v{
                  // coordinates     texture    norm            tangent
                  // x    y    z     u    v     x    y    z     x    y    z
                  0.f, 0.f, 1.f,  0.f, 1.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                  0.f, 0.f, 0.f,  0.f, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                  1.f, 0.f, 0.f,  1.f, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,

                  0.f, 0.f, 1.f,  0.f, 1.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                  1.f, 0.f, 0.f,  1.f, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                  1.f, 0.f, 1.f,  1.f, 1.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
          }, skybox(nullptr), need_upload(false)
{}

scene::EmptyScene::~EmptyScene()
{
    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(1, vbo);
    glDeleteTextures(4, texture);
    delete skybox;
}

void scene::EmptyScene::init(Scene &scene)
{
    if (skybox == nullptr)
    {
        skybox = new SkyBox(ASSET_PATH "/texture/skybox/right.jpg",
                            ASSET_PATH "/texture/skybox/left.jpg",
                            ASSET_PATH "/texture/skybox/top.jpg",
                            ASSET_PATH "/texture/skybox/bottom.jpg",
                            ASSET_PATH "/texture/skybox/back.jpg",
                            ASSET_PATH "/texture/skybox/front.jpg");

        glGenVertexArrays(1, vao);
        glGenBuffers(1, vbo);
        glGenTextures(4, texture);

        int w, h, ch;
        auto ptr = stbi_load(ASSET_PATH "/texture/floor6_d.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(texture[0], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
        ptr = stbi_load(ASSET_PATH "/texture/floor6_n.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(texture[1], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
        ptr = stbi_load(ASSET_PATH "/texture/floor6_s.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(texture[2], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
        ptr = stbi_load(ASSET_PATH "/texture/floor6_h.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(texture[3], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
    }

    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floor_v), nullptr, GL_STATIC_DRAW);
    ATTRIB_BIND_HELPER_WITH_TANGENT
}

void scene::EmptyScene::restart(Scene &scene)
{
    auto u = std::max(1, static_cast<int>(width / 2));
    auto v = std::max(1, static_cast<int>(height / 2));

    floor_v[25] = u; floor_v[47] = u; floor_v[58] = u;
    floor_v[4]  = v; floor_v[37] = v; floor_v[59] = v;
    floor_v[22] = width; floor_v[44] = width; floor_v[55] = width;
    floor_v[2]  = height; floor_v[35] = height; floor_v[57] = height;

    scene.character.reset(0.f, scene.character.characterHeight(), 0.f, 135.f, 15.f);
    scene.character.setFloating(false);
    scene.opt->gravity() = gravity;
    scene.opt->resistance() = resistance;

    need_upload = true;
}

void scene::EmptyScene::upload(Shader &scene_shader)
{
    if (!need_upload) return;

    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(floor_v), floor_v);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    scene_shader.activate();
    scene_shader.set("headlight.ambient", glm::vec3(.6f, .5f, .3f));
    scene_shader.set("headlight.diffuse", glm::vec3(.25f, .2f, .125f));
    scene_shader.set("headlight.specular", glm::vec3(.5f, .5f, .5f));
    scene_shader.set("headlight.coef_a0", 1.f);
    scene_shader.set("headlight.coef_a1", .09f);
    scene_shader.set("headlight.coef_a2", .032f);
    scene_shader.set("global_ambient", glm::vec3(.3f, .3f, .3f));
    scene_shader.activate(false);

    need_upload = false;
}

void scene::EmptyScene::render(Shader &scene_shader)
{
    scene_shader.activate();
    scene_shader.set("use_tangent", 1);
    scene_shader.set("material.parallel_height", 0.f);
    scene_shader.set("material.shininess", 32.f);
    scene_shader.set("material.ambient", glm::vec3(1.f, 1.f, 1.f));
    scene_shader.set("material.displace_amp", 0.f);
    scene_shader.set("material.displace_mid", 0.5f);

    glBindVertexArray(vao[0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, texture[2]);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, texture[3]);

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    scene_shader.activate(false);
}

void scene::EmptyScene::render()
{
    skybox->render();

    App::instance()->text("Single Click to shoot bounce ball",
                          App::instance()->frameWidth() - 10, App::instance()->frameHeight() - 25, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}
