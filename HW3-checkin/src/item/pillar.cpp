#include <glm/gtc/matrix_transform.hpp>
#include "item/pillar.hpp"
#include "global.hpp"

#include "stb_image.hpp"
#include "pillar.hpp"

using namespace px;

ItemInfo item::Pillar::ITEM_INFO("Pillar", "", 0, false, false, false);

item::Pillar::Pillar(glm::vec3 const &pos, float radius, float height)
        : Item(regItem()), vao{0}, vbo{0}
{
    place(pos);
    scale(glm::vec3(radius, height, radius));

    color.ambient = glm::vec3(0.3f, 0.f, 0.f);
    color.diffuse = glm::vec3(0.5f, 0.f, 0.f);
    color.specular = glm::vec3(1.f);
    color.shininess = 32.f;
}

item::Pillar::~Pillar()
{
    glDeleteVertexArrays(3, vao);
    glDeleteBuffers(6, vbo);
    vao[0] = 0; vbo[0] = 0; vbo[1] = 0;
    vao[1] = 0; vbo[2] = 0; vbo[3] = 0;
    vao[2] = 0; vbo[4] = 0; vbo[5] = 0;
}

Shader *item::Pillar::shader = nullptr;

void item::Pillar::initShader()
{
    if (shader == nullptr)
    {
        shader = new Shader(
#include "shader/glsl/simple_phong.vs"
        ,
#include "shader/glsl/simple_phong.fs"
        );
    }
}

void item::Pillar::destroyShader()
{
}

std::size_t item::Pillar::regItem()
{
    if (ITEM_INFO.id() == 0)
    {
        Item::reg(item::Pillar::ITEM_INFO, item::Pillar::create);
        if (ITEM_INFO.id() == 0)
            err("Failed to register Item: " + ITEM_INFO.name);
    }
    return ITEM_INFO.id();
}

std::shared_ptr<Item> item::Pillar::create()
{
    return std::shared_ptr<Item>(new Pillar());
}

void item::Pillar::init()
{
    initShader();

    glDeleteVertexArrays(3, vao);
    glDeleteBuffers(6, vbo);
    glGenVertexArrays(3, vao);
    glGenBuffers(6, vbo);

    std::vector<float> vertex, norm;
    constexpr auto grid = 32;
    constexpr auto gap = static_cast<float>(2*M_PI / grid);

    vertex.reserve(grid * 6);
    norm.reserve(grid * 6);
    float x, y;
    for (auto i = 0; i < grid+1; ++i) {
        x = std::cos(i*gap);
        y = std::sin(i*gap);

        vertex.push_back(x);
        vertex.push_back(-1.f);
        vertex.push_back(y);
        norm.push_back(x);
        norm.push_back(0.f);
        norm.push_back(y);

        vertex.push_back(x);
        vertex.push_back(1.f);
        vertex.push_back(y);
        norm.push_back(x);
        norm.push_back(0.f);
        norm.push_back(y);
    }
    glBindVertexArray(vao[0]);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, norm.size()*sizeof(float), norm.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    vertex.clear(); vertex.reserve(3*(grid+2));
    norm.clear(); norm.reserve(3);
    vertex.push_back(0.f);
    vertex.push_back(1.f);
    vertex.push_back(0.f);
    norm.push_back(0.f);
    norm.push_back(1.f);
    norm.push_back(0.f);
    for (auto i = 0; i < grid+1; ++i)
    {
        x = std::cos(i*gap);
        y = std::sin(i*gap);

        vertex.push_back(x);
        vertex.push_back(1.f);
        vertex.push_back(y);
    }
    glBindVertexArray(vao[1]);
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, norm.size()*sizeof(float), norm.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    norm[1] = -1.f;
    for (auto i = 1; i < 3*(grid+2); i+=3)
        vertex[i] = -1.f;
    glBindVertexArray(vao[2]);
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[4]);
    glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[5]);
    glBufferData(GL_ARRAY_BUFFER, norm.size()*sizeof(float), norm.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    shader->activate(false);
    _n_indices = vertex.size()/3;
}

void item::Pillar::render()
{
    shader->activate();
    auto model = glm::scale(glm::translate(glm::mat4(), pos()), scal());
    shader->set("model", model);

    shader->set("material.ambient", color.ambient);
    shader->set("material.diffuse", color.diffuse);
    shader->set("material.specular", color.specular);
    shader->set("material.shininess", color.shininess);
    shader->set("material.alpha", 1.f);

    glBindVertexArray(vao[2]);
    glDrawArrays(GL_TRIANGLE_FAN, 0, _n_indices);
    glBindVertexArray(vao[0]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, _n_indices*2);
    glBindVertexArray(vao[1]);
    glDrawArrays(GL_TRIANGLE_FAN, 0, _n_indices);

    glBindVertexArray(0);
    shader->activate(false);
}
