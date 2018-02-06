#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <iostream>

#include "particle_system/simple_2d.hpp"

using namespace px;

Simple2DParticleSystem::Simple2DParticleSystem()
        : ParticleSystem(),
          vao(0), vbo{0}, vbo_uvs{0}, texture(0),
          shader(nullptr)
{}

Simple2DParticleSystem::~Simple2DParticleSystem()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(4, vbo);
    glDeleteBuffers(2, vbo_uvs);

    delete shader;
}

void Simple2DParticleSystem::init(float *vertex, unsigned int v_count,
                                  unsigned int tex, float *uv, bool atlas)
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(4, vbo);
    glDeleteBuffers(2, vbo_uvs);

    debt_particles = 0.f;
    n_vertices = v_count;

    std::vector<Particle_t>(max_particles).swap(particles);

    if (shader == nullptr)
    {
        shader = new Shader(
#include "shader/glsl/simple_2d_particle.vs"
                ,
#include "shader/glsl/simple_particle.fs"
                           );
        shader->bind("GlobalAttributes", 0);
    }
    shader->activate();

    glGenVertexArrays(1, &vao);
    glGenBuffers(4, vbo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);  // vertex
    glBufferData(GL_ARRAY_BUFFER, n_vertices*2*sizeof(float), vertex, GL_STATIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(5);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);  // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);  // size + rotation
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]); // color
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(2);

    activateTexture(tex, uv, atlas);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    shader->activate(false);
}

void Simple2DParticleSystem::restart()
{
    std::vector<Particle_t>(max_particles).swap(particles);
    std::vector<float>(max_particles, 0.f).swap(life);
    std::vector<float>(particles.size() * 3).swap(position);
    std::vector<float>(particles.size() * 2).swap(size_rot);
    std::vector<float>(particles.size() * 4).swap(color);
    if (use_atlas)
        std::vector<float>(particles.size() * 2).swap(uv_offset);
    else
        uv_offset.clear();

    need_upload = true;
}

void Simple2DParticleSystem::activateTexture(unsigned int tex, float *uv, bool atlas)
{
    if (shader == nullptr)
        err("Failed to activate texture for particle system without initialization");

    if ((tex == 0 && uv != nullptr) || (tex != 0 && uv == nullptr))
        err("Both of texture id and uv coordinate are needed to bind texture.");

    texture = tex;
    shader->set("use_texture", texture == 0 ? 0 : 1);

    glDeleteBuffers(1, &vbo_uvs[0]);
    if (texture != 0)
    {
        shader->activate();
        shader->set("sprite", 0);
        shader->activate(false);
        glGenBuffers(1, &vbo_uvs[0]);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_uvs[0]);  // uv mapping
        glBufferData(GL_ARRAY_BUFFER, n_vertices*2*sizeof(float), uv, GL_STATIC_DRAW);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
        glEnableVertexAttribArray(3);
        activateAtlas(atlas);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void Simple2DParticleSystem::activateAtlas(bool atlas)
{
    if (shader == nullptr)
        err("Failed to activate texture atlas for particle system without initialization");
    if (texture == 0 && atlas)
        err("Failed to activate texture atlas for particle system without activated texture");

    use_atlas = atlas;
    shader->activate();
    shader->set("use_atlas", use_atlas == 0 ? 0 : 1);
    shader->activate(false);
    glDeleteBuffers(1, &vbo_uvs[1]);
    if (use_atlas)
    {
        glGenBuffers(1, &vbo_uvs[1]);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_uvs[1]); // atlas
        glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
        glEnableVertexAttribArray(4);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void Simple2DParticleSystem::upload()
{
    if (need_upload)
    {
        glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, position.size()*sizeof(float), position.data(), GL_STREAM_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glBufferData(GL_ARRAY_BUFFER, size_rot.size()*sizeof(float), size_rot.data(), GL_STREAM_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
        glBufferData(GL_ARRAY_BUFFER, color.size()*sizeof(float), color.data(), GL_STREAM_DRAW);

        if (use_atlas)
        {
            glBindBuffer(GL_ARRAY_BUFFER, vbo_uvs[1]);
            glBufferData(GL_ARRAY_BUFFER, uv_offset.size()*sizeof(float), uv_offset.data(), GL_STREAM_DRAW);
        }
        need_upload = false;
    }
    else
    {
        glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, count()*3*sizeof(float), position.data());

        glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, count()*2*sizeof(float), size_rot.data());

        glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, count()*4*sizeof(float), color.data());

        if (use_atlas)
        {
            glBindBuffer(GL_ARRAY_BUFFER, vbo_uvs[1]);
            glBufferSubData(GL_ARRAY_BUFFER, 0, count()*2*sizeof(float), uv_offset.data());
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Simple2DParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (position.size() != total() * 3)
        position.resize(total() * 3);
    if (size_rot.size() != total() * 2)
        size_rot.resize(total() * 2);
    if (color.size() != total() * 4)
        color.resize(total() * 4);
    if (use_atlas && uv_offset.size() != total() * 2)
        uv_offset.resize(total() * 2);

    std::vector<std::pair<int, int> > valid_indices;
    valid_indices.reserve(particles.size());

    auto tot = static_cast<int>(particles.size());
#pragma omp parallel for num_threads(8)
    for (auto i = 0; i < tot; ++i)
    {
        if (life[i] > 0.f)
        {
            auto &p = particles[i];
            update_fn(p, life[i], dt, i);
//            if (update_fn(p, life[i], dt, i))
//                valid_indices.emplace_back(i, cam_pos ? glm::distance2(p.position, *cam_pos) : 0.f);
        }
    }

    debt_particles += std::min(0.05f, dt) * birth_rate;
    auto new_particles = static_cast<int>(debt_particles);
//    auto new_particles = std::min(static_cast<int>(debt_particles),
//                                  static_cast<int>(particles.size() - valid_indices.size()));
    debt_particles -= new_particles;
    auto generated = 0;
    for (auto i = 0, tot = static_cast<int>(particles.size()); i < tot; ++i)
    {
        auto &p = particles[i];
        if (life[i] <= 0.f)
        {
            if (generated < new_particles && spawn_fn(p, life[i], i))
//            if (spawn_fn(p, life[i], i))
            {
                valid_indices.emplace_back(i, cam_pos ? glm::distance2(p.position, *cam_pos) : 0.f);
                ++generated;
//                if (++generated == new_particles)
//                    break;
            }
        }
        else
            valid_indices.emplace_back(i, cam_pos ? glm::distance2(p.position, *cam_pos) : 0.f);
    }

    if (cam_pos)
    {
        std::sort(valid_indices.begin(), valid_indices.end(),
                  [&](std::pair<int, int> const& lhs, std::pair<int, int> const& rhs)
                  {
                      if (lhs.second == rhs.second)
                          return life[lhs.first] > life[rhs.first];
                      return lhs.second > rhs.second;
                  });
    }
    n_particles = static_cast<unsigned int>(valid_indices.size());

#pragma omp parallel for num_threads(8)
    for (unsigned int i = 0; i < n_particles; ++i)
    {
        auto &p = particles[valid_indices[i].first];
        auto idx = i + i;

        size_rot[idx]   = p.size;
        size_rot[idx+1] = p.rotation;
        if (use_atlas)
        {
            uv_offset[idx] = p.u_offset;
            uv_offset[idx+1] = p.v_offset;
        }

        idx += i;
        position[idx]   = p.position.x;
        position[idx+1] = p.position.y;
        position[idx+2] = p.position.z;

        idx += i;
        color[idx]   = p.color.r;
        color[idx+1] = p.color.g;
        color[idx+2] = p.color.b;
        color[idx+3] = p.color.a;
    }
}

void Simple2DParticleSystem::render(GLenum gl_draw_mode)
{
    shader->activate();

    glBindVertexArray(vao);
    glVertexAttribDivisor(5, 0); // vertex             a group for each instance
    glVertexAttribDivisor(0, 1); // position           one for each instance
    glVertexAttribDivisor(1, 1); // size+rotation
    glVertexAttribDivisor(2, 1); // color
    if (texture != 0)
    {
        glVertexAttribDivisor(3, 0); // u, v          for each vertex
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        if (use_atlas)
            glVertexAttribDivisor(4, 1); // u_offset, v_offset
    }

    glDrawArraysInstanced(gl_draw_mode, 0, n_vertices, count());

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    shader->activate(false);
}
