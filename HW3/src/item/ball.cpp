#include <glm/gtc/matrix_transform.hpp>
#include "item/ball.hpp"
#include "global.hpp"

#include "stb_image.hpp"
#include "pillar.hpp"

using namespace px;

ItemInfo item::Ball::ITEM_INFO("Ball", "", 0, false, false, false);

item::Ball::Ball(glm::vec3 const &pos, float radius)
        : Item(regItem()), vao{0}, vbo{0}
{
    place(pos);
    scale(glm::vec3(radius, radius, radius));

    color.ambient = glm::vec3(0.0f, 0.3f, 0.f);
    color.diffuse = glm::vec3(0.0f, 0.5f, 0.f);
    color.specular = glm::vec3(1.f);
    color.shininess = 32.f;
}

item::Ball::~Ball()
{
    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(2, vbo);
    vao[0] = 0; vbo[0] = 0; vbo[1] = 0;
}

Shader *item::Ball::shader = nullptr;

void item::Ball::initShader()
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

void item::Ball::destroyShader()
{
}

std::size_t item::Ball::regItem()
{
    if (ITEM_INFO.id() == 0)
    {
        Item::reg(item::Ball::ITEM_INFO, item::Ball::create);
        if (ITEM_INFO.id() == 0)
            err("Failed to register Item: " + ITEM_INFO.name);
    }
    return ITEM_INFO.id();
}

std::shared_ptr<Item> item::Ball::create()
{
    return std::shared_ptr<Item>(new Ball());
}

void item::Ball::init()
{
    initShader();

    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(2, vbo);
    glGenVertexArrays(1, vao);
    glGenBuffers(2, vbo);

    unsigned int n_phi = 48;
    auto n_theta = 2 * n_phi;
    auto tot_point = n_theta * n_phi * 3;
    auto n_indices = n_theta * (n_phi - 1) * 6;

    float x, y, z;
    std::vector<float> sphere(tot_point);

    auto phi = 0.f, theta = 0.f;
    auto d_theta = 2*static_cast<float>(M_PI) / n_theta;
    auto d_phi = static_cast<float>(M_PI) / (n_phi-1);
    for (unsigned int idx = 0, i = 0; i < n_theta; ++i)
    {
        for (unsigned int j = 0; j < n_phi; ++j)
        {
            x = std::sin(phi) * std::cos(theta);
            y = std::sin(phi) * std::sin(theta);
            z = std::cos(phi);

            sphere[idx++] = x;
            sphere[idx++] = y;
            sphere[idx++] = z;

            phi += d_phi;
        }
        theta += d_theta;
    }

    std::vector<unsigned int> vertex_order(n_indices);
    for (unsigned int idx = 0, v_idx = 0, j = 1; j < n_phi; ++j)
    {
        for (unsigned int i = 1; i < n_theta; ++i)
        {
            vertex_order[idx++] = v_idx;
            vertex_order[idx++] = v_idx + 1;
            vertex_order[idx++] = v_idx + n_theta;

            vertex_order[idx++] = v_idx + n_theta;
            vertex_order[idx++] = v_idx + 1;
            vertex_order[idx++] = v_idx + n_theta + 1;

            ++v_idx;
        }

        vertex_order[idx++] = v_idx;
        vertex_order[idx++] = v_idx + 1 - n_theta;
        vertex_order[idx++] = v_idx + n_theta;

        vertex_order[idx++] = v_idx + n_theta;
        vertex_order[idx++] = v_idx - n_theta + 1;
        vertex_order[idx++] = v_idx + 1;

        ++v_idx;
    }

    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*tot_point, sphere.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*n_indices, vertex_order.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    shader->activate(false);

    _n_indices = n_indices;
}

void item::Ball::render()
{
    shader->activate();
    auto model = glm::scale(glm::translate(glm::mat4(), pos()), scal());
    shader->set("model", model);

    shader->set("material.ambient", color.ambient);
    shader->set("material.diffuse", color.diffuse);
    shader->set("material.specular", color.specular);
    shader->set("material.shininess", color.shininess);
    shader->set("material.alpha", 1.f);

    glBindVertexArray(vao[0]);
    glDrawElements(GL_TRIANGLES, _n_indices, GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
    shader->activate(false);
}
