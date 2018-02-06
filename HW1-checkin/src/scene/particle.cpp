#include "scene/particle.hpp"
#include "scene.hpp"
#include "util/random.hpp"

#include <cstring>

using namespace px;

scene::Particle::Particle()
        : BaseScene(),
          gravity(0.f, -5.f, 0.f),
          max_particles(1000000), birth_rate(60000),
          vao(0), vbo{0},
          shader(nullptr), text(nullptr)
{}

scene::Particle::~Particle()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(3, vbo);
    delete shader;
}

void scene::Particle::init(Scene &scene)
{
    auto grid = 32;

    std::vector<float> vertex;
    vertex.reserve(grid*4);
    auto gap = 2.f / grid;
    for (float x = -1; x < 1; x += gap)
    {
        vertex.push_back(x);
        vertex.push_back(-std::sqrt(1 - x*x));
    }
    for (float x = 1; x > -1; x -= gap)
    {
        vertex.push_back(x);
        vertex.push_back(std::sqrt(1 - x*x));
    }

    n_vertex = static_cast<int>(vertex.size());
    debt_particles = 0.f;

    std::vector<Particle_t>(max_particles).swap(particles);
    std::vector<float>(max_particles, 0.f).swap(life);
    std::vector<float>(max_particles * 4, 0.f).swap(position);
    std::vector<float>(max_particles * 4, 0.f).swap(color);

    glGenVertexArrays(1, &vao);
    glGenBuffers(3, vbo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, n_vertex*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, position.size()*sizeof(float), nullptr, GL_STREAM_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, color.size()*sizeof(float), nullptr, GL_STREAM_DRAW);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void *)0);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glClearColor(0.f, 0.f, 0.f, 1.f);

    if (shader == nullptr)
    {
        shader = new Shader(
#include "shader/glsl/simple_2d_particle.vs"
                ,
#include "shader/glsl/simple_2d_particle.fs"
                           );
        shader->bind("GlobalAttributes", 0);
    }
    if (text == nullptr)
    {
        text = new TextShader;
        text->setFontHeight(static_cast<std::size_t>(40));
        static const unsigned char TITLE_FONT_DATA[] = {
#include "font/Just_My_Type.dat"
        };
        text->addFont(TITLE_FONT_DATA, sizeof(TITLE_FONT_DATA));
    }
}

void scene::Particle::restart(Scene &scene)
{
    scene.character.setShootable(false);
    scene.character.reset(0.f, scene.character.characterHeight(), 0.f, 0.f, 0.f);
    scene.opt->gravity() = gravity;

    std::memset(life.data(), 0, life.size()*sizeof(float));
}

void scene::Particle::upload(Shader &scene_shader)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, count()*4*sizeof(float), position.data());

    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, count()*4*sizeof(float), color.data());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void scene::Particle::render(Shader &scene_shader)
{

}

void scene::Particle::update(float dt)
{
    auto field_height = 0.f;
    glm::vec3 field_norm(0.f, 1.f, 0.f);

    n_particles = 0;

    if (position.size() != total() * 4)
        position.resize(total() * 4);
    if (color.size() != total() * 4)
        color.resize(total() * 4);

    auto idx = 0, index = 0;
    for (auto i = 0, tot = static_cast<int>(particles.size()); i < tot; ++i)
    {
        auto &p = particles[i];

        if (life[i] > 0.f)
            life[i] -= dt;

        if (life[i] <= 0.f)
            continue;

        auto h = field_height + p.size;
        auto above_field = p.position.y >= h;
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;
        if (above_field && p.position.y < h )
        {
            p.position.y = h;
            p.velocity = glm::reflect(p.velocity, field_norm);
            p.velocity *= 0.95f;

            if (std::abs(p.velocity.x) < 1e-4) p.velocity.x = 0;
            if (std::abs(p.velocity.y) < 1e-2) p.velocity.y = 0;
            if (std::abs(p.velocity.z) < 1e-4) p.velocity.z = 0;

            if (p.velocity.y == 0 && life[i] > 1.f)
                life[i] = rnd()*.25f + .75f; // FIXME random life makes it inaccurate to count valid particles by start and end indices
        }

        position[idx]   = p.position.x;
        position[idx+1] = p.position.y;
        position[idx+2] = p.position.z;
        position[idx+3] = p.size;

        color[idx]   = p.color.r;
        color[idx+1] = p.color.g;
        color[idx+2] = p.color.b;
        color[idx+3] = p.color.a;

        idx += 4;
        ++n_particles;

        if (index < 0) index = i;
    }

    if (birth_rate == 0)
        return;

    debt_particles += std::min(0.05f, dt) * birth_rate;
    auto new_particles = std::min(static_cast<int>(debt_particles),
                                   std::max(0, static_cast<int>(particles.size() - n_particles)));
    debt_particles -= new_particles;

    if (n_particles > 0)
    {
        auto space = particles.size() - (index + n_particles);
        if (space < 0) index += space - new_particles;
        if (index < 0)
        {
            index = 0;
            n_particles = particles.size();
        }
        else
            n_particles += new_particles;
    }
    else
    {
        index = 0;
        n_particles += new_particles;
    }

    for (auto end = index + static_cast<int>(n_particles); index < end; ++index)
    {
        auto & p = particles[index];

        if (life[index] > 0.f)
            continue;

        life[index] = 100.f;

        p.color.r = rnd() * .5f + .5f;
        p.color.g = rnd() * .5f + .5f;
        p.color.b = rnd() * .5f + .5f;
        p.color.a = rnd() * .5f + .5f;

        p.position.x = 0.f;
        p.position.y = .01f;
        p.position.z = -5.f;
        p.size = .0075f;

        p.velocity.x = rnd()*.25f;
        p.velocity.y = 3.f + rnd();
        p.velocity.z = rnd()*.25f;

        position[idx]   = p.position.x;
        position[idx+1] = p.position.y;
        position[idx+2] = p.position.z;
        position[idx+3] = p.size;

        color[idx]   = p.color.r;
        color[idx+1] = p.color.g;
        color[idx+2] = p.color.b;
        color[idx+3] = p.color.a;

        idx += 4;
    }
}

void scene::Particle::render()
{
    shader->use();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(vao);
    glVertexAttribDivisor(0, 0);
    glVertexAttribDivisor(1, 1);
    glVertexAttribDivisor(2, 2);
    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, n_vertex, count());

    glDisable(GL_BLEND);
    glBindVertexArray(0);

    text->render("Particles: " + std::to_string(n_particles) + "/" + std::to_string(particles.size()),
                 10, 10, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);
    text->render("Mode: instancing",
                 10, 30, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);
}