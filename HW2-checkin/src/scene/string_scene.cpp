#include "scene/string_scene.hpp"
#include "scene.hpp"
#include "global.hpp"
#include "app.hpp"

#include "stb_image.hpp"

using namespace px;

class scene::StringScene::impl
{
public:
    const char *VS = "#version 420 core\n"
        "layout (location = 5) in vec2 vertices;"
        "layout (location = 0) in vec3 pos;"
        ""
        "layout (std140, binding = 0) uniform GlobalAttributes"
        "{"
        "   mat4 view;"
        "   mat4 proj;"
        "   vec3 cam_pos;"
        "};"
        ""
        "void main()"
        "{"
        "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]);"
        "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]);"
        "	vec3 vertex_pos = pos"
        "	        + (right_vec * vertices.x +  up_vec * vertices.y) * .25;"
        "	gl_Position = proj * view * vec4(vertex_pos, 1.0f);"
        "}";
    const char *FS = "#version 330 core\n"
        ""
        "out vec4 color;"
        ""
        "void main()"
        "{"
        "   color = vec4(1.f, 1.f, 1.f, 1.f);"
        "}";

    unsigned int n_vertices;
    unsigned int n_particles;

    struct Particle_t
    {
        glm::vec3 position;
        glm::vec3 velocity;
        float mass;
    };
    struct Spring
    {
        std::pair<unsigned int, unsigned int> knots,
        float length;
    }
    std::vector<Particle_t> particles;
    std::vector<Spring_t> springs;
};

scene::StringScene::StringScene()
        : BaseScene(),
          vao{0}, vbo{0}, shader(nullptr),
          pimpl(new impl)
{}

scene::StringScene::~StringScene()
{
    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(2, vbo);
    delete shader;
}

void scene::StringScene::init(Scene &scene)
{
    if (shader == nullptr)
        shader = new Shader(pimpl->VS, pimpl->FS);

    glDeleteVertexArrays(1, vao);
    glDeleteBuffers(2, vbo);

    glGenVertexArrays(1, vao);
    glGenBuffers(2, vbo);

    std::vector<float> vertex;
    constexpr auto grid = 32;
    constexpr auto gap = static_cast<float>(M_PI / grid);
    const auto c = std::cos(gap);
    const auto s = std::sin(gap);
    vertex.reserve(grid * 4);
    vertex.push_back(1.f);
    vertex.push_back(0.f);
    auto x = 1.f;
    auto y = 0.f;
    for (auto i = 1; i < grid; ++i) {
        float tmp_x = c * x - s * y;
        y = s * x + c * y;
        x = tmp_x;
        vertex.push_back(x);
        vertex.push_back(y);
        vertex.push_back(x);
        vertex.push_back(-y);
    }
    vertex.push_back(-1.f);
    vertex.push_back(0.f);
    pimpl->n_vertices = static_cast<unsigned int>(vertex.size());

    shader->activate();
    shader->bind("GlobalAttributes", 0);

    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*pimpl->n_vertices, vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(5);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    shader->activate(false);
}

void scene::StringScene::restart(Scene &scene)
{
    scene.character.reset(0.f, scene.character.characterHeight(), 0.f, 135.f, 15.f);
    scene.character.setFloating(true);

    pimpl->particles.clear();
    pimpl->particles.push_back({glm::vec3(1.f, 0.f, 1.f)});
    pimpl->n_particles = static_cast<unsigned int>(pimpl->particles.size());
}

void scene::StringScene::upload(Shader &scene_shader)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, pimpl->n_particles*3*sizeof(float), pimpl->particles.data());
}

void scene::StringScene::update(float dt)
{
    for (auto & s: pimpl->springs)
    {
        auto F = pimpl->particles[s.knots.first].position - pimpl->particles[s.knots.second].position;
        auto x = glm::length(F);
        F /= x;
        F *= - k/s.length * (x - s.length); // k is prop to the natural length of spring
        F += - kv * (pimpl->particles[s.knots.first].velocity - pimpl->particles[s.knots.second].velocity);

        pimpl->particles[s.knots.first].force += F;
        pimpl->particles[s.knots.second].force -= F;
    }

    for (auto & p: pimpl->particles)
    {
        if (p.mass == std::numeric_limits<float>::max()) // fixed point with infinite mass
            continue;

        auto acc = gravity + p.force/p.mass;
        p.velocity += acc * dt;
        p.position += p.velocity * dt;
    }
}

void scene::StringScene::render(Shader &scene_shader)
{}

void scene::StringScene::render()
{
    shader->activate();
    glBindVertexArray(vao[0]);
    glVertexAttribDivisor(5, 0);
    glVertexAttribDivisor(0, 1);

    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, pimpl->n_vertices, pimpl->n_particles);
}
