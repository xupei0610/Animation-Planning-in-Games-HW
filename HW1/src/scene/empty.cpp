#include "scene/empty.hpp"
#include "scene.hpp"
#include "global.hpp"

#include "stb_image.hpp"

using namespace px;

scene::Empty::Empty()
        : BaseScene(), gravity(0.f, -5.f, 0.f), resistance(.5f),
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

scene::Empty::~Empty()
{
    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(1, vbo);
    glDeleteTextures(4, texture);
    delete skybox;
}

void scene::Empty::init(Scene &scene)
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
    ATTRIB_BIND_HELPER_WITH_TANGENT
}

void scene::Empty::restart(Scene &scene)
{
    constexpr auto u = 25;
    constexpr auto v = 25;
    constexpr auto w = 50;
    constexpr auto h = 50;

    floor_v[25] = u; floor_v[47] = u; floor_v[58] = u;
    floor_v[4]  = v; floor_v[37] = v; floor_v[59] = v;
    floor_v[22] = w; floor_v[44] = w; floor_v[55] = w;
    floor_v[2]  = h; floor_v[35] = h; floor_v[57] = h;

    scene.character.reset(50.f, scene.character.characterHeight(), 50.f, 0.f, 0.f);
    scene.opt->gravity() = gravity;
    scene.opt->resistance() = resistance;

    need_upload = true;
}

void scene::Empty::upload(Shader &scene_shader)
{
    if (!need_upload) return;

    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floor_v), floor_v, GL_STATIC_DRAW);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    scene_shader.use();
    scene_shader.set("headlight.ambient", glm::vec3(.6f, .5f, .3f));
    scene_shader.set("headlight.diffuse", glm::vec3(.25f, .2f, .125f));
    scene_shader.set("headlight.specular", glm::vec3(.5f, .5f, .5f));
    scene_shader.set("headlight.coef_a0", 1.f);
    scene_shader.set("headlight.coef_a1", .09f);
    scene_shader.set("headlight.coef_a2", .032f);
    scene_shader.set("global_ambient", glm::vec3(.3f, .3f, .3f));

    glClearColor(.2f, .3f, .3f, 1.f);

    need_upload = false;
}

void scene::Empty::render(Shader &scene_shader,
                           glm::mat4 const &view, glm::mat4 const &proj)
{
    scene_shader.use();
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
}

void scene::Empty::render(glm::mat4 const &view, glm::mat4 const &proj)
{
    skybox->render(view, proj);
}
