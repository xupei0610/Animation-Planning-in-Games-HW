#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <cstring>
#include "item/light_ball.hpp"
#include "util/random.hpp"

using namespace px;

ItemInfo item::LightBall::ITEM_INFO("Light Ball", "", 0, false, false, true);

item::LightBall::LightBall(const glm::vec3 &p, const glm::vec3 &s,
                            float m, const glm::vec3 &v, const glm::vec3 &a)
    : Item(regItem()), Rigid(m, v, a)
{
    place(p);
    scale(s);

    setGrid(18);
    
    _light.ambient  = glm::vec3(.5f);
    _light.diffuse  = glm::vec3(.5f);
    _light.specular = glm::vec3(.5f);
    _light.coef     = glm::vec3(1.f, 1.f, 2.f);
}

item::LightBall::~LightBall()
{}

unsigned int item::LightBall::vao = 0;
unsigned int item::LightBall::vbo[2] = {0, 0};
Shader *item::LightBall::shader = nullptr;

unsigned int item::LightBall::initShader(unsigned int n_theta)
{
    if (shader == nullptr)
    {
        constexpr auto vs =
#include "shader/glsl/lamp_shader.vs"
        ;
        constexpr auto fs =
#include "shader/glsl/lamp_shader.fs"
        ;

        shader = new Shader(vs, fs);
        shader->bind("GlobalAttributes", 0);
        glGenVertexArrays(1, &vao);
        glGenBuffers(2, vbo);
    }

    auto n_phi = 2 * n_theta;
    auto tot_point = n_phi * n_theta * 3;
    auto n_indices = n_phi * (n_theta - 1) * 6;

    float x, y, z;
    auto sphere = new float[tot_point];

    auto theta = 0.f, phi = 0.f;
    float d_phi = 2 * M_PI / n_phi;
    float d_theta = M_PI / (n_theta - 1);
    for (unsigned int idx = 0, i = 0; i < n_phi; ++i)
    {
        for (unsigned int j = 0; j < n_theta; ++j)
        {
            x = std::sin(phi) * std::cos(theta);
            y = std::sin(phi) * std::sin(theta);
            z = std::cos(phi);

            sphere[idx++] = x;
            sphere[idx++] = y;
            sphere[idx++] = z;

            theta += d_theta;
        }
        phi += d_phi;
    }

    auto vertex_order = new unsigned int[n_indices];
    for (unsigned int idx = 0, v_idx = 0, j = 1; j < n_theta; ++j)
    {
        for (unsigned int i = 1; i < n_phi; ++i)
        {
            vertex_order[idx++] = v_idx;
            vertex_order[idx++] = v_idx + 1;
            vertex_order[idx++] = v_idx + n_phi;

            vertex_order[idx++] = v_idx + n_phi;
            vertex_order[idx++] = v_idx + 1;
            vertex_order[idx++] = v_idx + n_phi + 1;

            ++v_idx;
        }

        vertex_order[idx++] = v_idx;
        vertex_order[idx++] = v_idx + 1 - n_phi;
        vertex_order[idx++] = v_idx + n_phi;

        vertex_order[idx++] = v_idx + n_phi;
        vertex_order[idx++] = v_idx - n_phi + 1;
        vertex_order[idx++] = v_idx + 1;

        ++v_idx;
    }

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*tot_point, sphere, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*n_indices, vertex_order, GL_STATIC_DRAW);

    glBindVertexArray(0);
    delete [] sphere;
    delete [] vertex_order;

    return n_indices;
}

void item::LightBall::destroyShader()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(2, vbo);
    delete shader;

    vao = 0;
    vbo[0] = 0;
    vbo[1] = 0;
    shader = nullptr;
}

std::size_t item::LightBall::regItem()
{
    if (ITEM_INFO.id() == 0)
    {
        Item::reg(item::LightBall::ITEM_INFO, item::LightBall::create);
        if (ITEM_INFO.id() == 0)
            err("Failed to register Item: " + ITEM_INFO.name);
    }
    return ITEM_INFO.id();
}

std::shared_ptr<Item> item::LightBall::create()
{
    return std::shared_ptr<Item>(new LightBall());
}

void item::LightBall::update(float dt)
{
    _movement = step(dt);
}

void item::LightBall::hit(glm::vec3 const &at, glm::vec3 const &norm)
{
    velocity() = glm::reflect(velocity(), norm);
    velocity() *= 0.95f;

    if (std::abs(velocity().x) < 1e-4) velocity().x = 0;
    if (std::abs(velocity().y) < 1e-4) velocity().y = 0;
    if (std::abs(velocity().z) < 1e-4) velocity().z = 0;
}

void item::LightBall::init()
{
    _n_indices = initShader(grid());
}

void item::LightBall::render()
{
    auto model = glm::scale(glm::translate(glm::mat4(), pos()), scal());

    shader->use();
    shader->set("diffuse", color());
    shader->set("model", model);

    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, _n_indices, GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
}

void item::LightBall::setGrid(unsigned int n)
{
    _n_theta = n;
}
