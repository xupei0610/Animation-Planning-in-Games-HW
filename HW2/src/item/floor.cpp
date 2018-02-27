#include <glm/gtc/matrix_transform.hpp>
#include "item/floor.hpp"
#include "global.hpp"

#include "stb_image.hpp"
using namespace px;

ItemInfo item::Floor::ITEM_INFO("Floor", "", 0, false, false, false);

item::Floor::Floor(glm::vec3 const &pos, float width, float height,
                   float u, float v)
        : Item(regItem()), vao(0), vbo(0)
{
    place(pos);
    scale(glm::vec3(width, 1.f, height));
    setGrid(u, v);
}

item::Floor::~Floor()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    vao = 0; vbo = 0;
}

unsigned int item::Floor::texture[4] = {0};

void item::Floor::initShader()
{
    if (texture[0] == 0)
    {
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
}

void item::Floor::destroyShader()
{
    glDeleteTextures(4, texture);
    texture[0] = 0; texture[1] = 0; texture[2] = 0; texture[3] = 0;
}

std::size_t item::Floor::regItem()
{
    if (ITEM_INFO.id() == 0)
    {
        Item::reg(item::Floor::ITEM_INFO, item::Floor::create);
        if (ITEM_INFO.id() == 0)
            err("Failed to register Item: " + ITEM_INFO.name);
    }
    return ITEM_INFO.id();
}

std::shared_ptr<Item> item::Floor::create()
{
    return std::shared_ptr<Item>(new Floor());
}

void item::Floor::init()
{
    initShader();

    if (vao == 0)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*66, nullptr, GL_STATIC_DRAW);
        ATTRIB_BIND_HELPER_WITH_TANGENT
        _need_upload = true;
    }
}

void item::Floor::render(Shader &scene_shader)
{
    if (_need_upload)
    {
        float floor_v[] = {
                // coordinates     texture    norm            tangent
                // x    y    z     u    v     x    y    z     x    y    z
                0.f, 0.f, 1.f,  0.f, _v,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                0.f, 0.f, 0.f,  0.f, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                1.f, 0.f, 0.f,  _u, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,

                0.f, 0.f, 1.f,  0.f, _v,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                1.f, 0.f, 0.f,  _u, 0.f,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
                1.f, 0.f, 1.f,  _u, _v,  0.f, 1.f, 0.f,  1.f, 0.f, 0.f,
        };
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(floor_v), floor_v);
    }
    scene_shader.activate();
    scene_shader.set("use_tangent", 1);
    scene_shader.set("material.parallel_height", 0.f);
    scene_shader.set("material.shininess", 32.f);
    scene_shader.set("material.ambient", glm::vec3(1.f, 1.f, 1.f));
    scene_shader.set("material.displace_amp", 0.f);
    scene_shader.set("material.displace_mid", 0.5f);
    auto model = glm::scale(glm::translate(glm::mat4(), pos()), scal());
    scene_shader.set("model", model);

    glBindVertexArray(vao);
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
