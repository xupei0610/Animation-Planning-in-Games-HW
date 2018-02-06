#ifndef PX_CG_SCENE_IMPL_BENCHMARK_TRANSFORM_FEEDBACK_PARTICLE_SYSTEM
#define PX_CG_SCENE_IMPL_BENCHMARK_TRANSFORM_FEEDBACK_PARTICLE_SYSTEM


scene::BenchmarkScene::TransformFeedbackParticleSystem::TransformFeedbackParticleSystem()
        : ParticleSystem(),
          vao{0}, vbo{0}, tfbo{0},
          compute_vertex_shader(nullptr), draw_shader(nullptr)
{}

scene::BenchmarkScene::TransformFeedbackParticleSystem::~TransformFeedbackParticleSystem()
{
    glDeleteVertexArrays(3, vao);
    glDeleteBuffers(4, vbo);
    glDeleteTransformFeedbacks(2, tfbo);

    delete compute_vertex_shader;
    delete draw_shader;
}

void scene::BenchmarkScene::TransformFeedbackParticleSystem::init(float *vertex, unsigned int v_count,
          unsigned int tex, float *uv, bool atlas)
{
    static const char COMPUTE_VS[] = "#version 330 core\n"
            "layout (location = 0) in vec3 position;"
            "layout (location = 1) in vec3 velocity;"
            "layout (location = 2) in vec4 color;"
            "layout (location = 3) in vec3 lifetime;"
            "layout (location = 4) in float life;"
            ""
            "out VS_OUT"
            "{"
            "   vec3 position;"
            "   vec3 velocity;"
            "   vec4 color;"
            "   vec3 lifetime;"
            "   float life;"
            "   vec3 gPosition;"
            "} particle;"
            ""
            "uniform float dt;"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            "const float double_PI = 2 * PI;"
            "const float drot = 0.06f * PI;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn(vec2 sd)"
            "{"
            "   float theta = PI * (rnd(sd) * 2.f - 1.f); ++sd.x;"
            "   float phi = acos(rnd(sd) * 2.f - 1.f);    ++sd.x;"
            "   float r0 = pow(rnd(sd) * 125.f, 1.f/3.f); ++sd.x;"
            "   particle.position = vec3(r0 * sin(phi) * cos(theta),"
            "                            r0 * cos(phi),"
            "                            r0 * sin(phi) * sin(theta));"
            "   particle.velocity = 0.0001f * particle.position;"
            "   particle.lifetime = vec3(4.f + (2*rnd(sd) - 1.f) * (4.f / 4),  (++sd.x,"
            "                        1.f + (2*rnd(sd) - 1.f) * (1.f / 4)), (++sd.x,"
            "                        5.f + (2*rnd(sd) - 1.f) * (5.f / 4)));"
            "   particle.life = particle.lifetime.x + particle.lifetime.y + particle.lifetime.z;"
            "   particle.color = vec4(1.f, .45f, .2f, 0.f);"
            "   particle.gPosition = particle.position;"
            ""
            "}"
            ""
            "void update(vec2 sd)"
            "{"
            ""
            "   if (particle.life > particle.lifetime.y + particle.lifetime.z)"
            "       particle.color.a += dt / particle.lifetime.x;"
            "   else if (particle.life > particle.lifetime.z)"
            "       particle.color.a = 1.f;"
            "   else"
            "   {"
            "       particle.color.a -= dt / particle.lifetime.z;"
            "       particle.velocity.x += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particle.velocity.y += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particle.velocity.z += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particle.position += particle.velocity;"
            "   }"
            ""
            "   float rotation = drot * (particle.lifetime.x+particle.lifetime.y+particle.lifetime.z - particle.life);"
            "   float rot_c = cos(rotation);"
            "   float rot_s = sin(rotation);"
            "   particle.gPosition = vec3(rot_c * particle.position.x + rot_s * particle.position.z,"
            "                             particle.position.y,"
            "                             rot_c * particle.position.z - rot_s * particle.position.x);"
            "}"
            ""
            "void main()"
            "{"
            "   if (dt < 0.f)"
            "   {"
            "       spawn(vec2(gl_VertexID, gl_VertexID));"
            "   }"
            "   else"
            "   {"
            "       vec2 sd = position.xy;"
            "       particle.life = life - dt;"
            "       if (particle.life > 0.f)"
            "       {"
            "           particle.position = position;"
            "           particle.velocity = velocity;"
            "           particle.color = color;"
            "           particle.lifetime = lifetime;"
            "           update(sd);"
            "       }"
            "       else"
            "           spawn(sd);"
            "   }"
            "}";

    if (compute_vertex_shader == nullptr)
    {
        static const char *feedback[] = {
                "VS_OUT.position",
                "VS_OUT.velocity",
                "VS_OUT.color",
                "VS_OUT.lifetime",
                "VS_OUT.life",
                "VS_OUT.gPosition"
        };
        compute_vertex_shader = new Shader(COMPUTE_VS, feedback, 6);
    }

    if (draw_shader == nullptr)
    {
        draw_shader = new Shader(
#include "shader/glsl/simple_2d_particle.vs"
                ,
#include "shader/glsl/simple_particle.fs"
                                );
        draw_shader->bind("GlobalAttributes", 0);
    }

    glDeleteVertexArrays(3, vao);
    glDeleteBuffers(4, vbo);
    glDeleteTransformFeedbacks(2, tfbo);

    glGenVertexArrays(3, vao);
    glGenBuffers(4, vbo);
    glGenTransformFeedbacks(2, tfbo);

    n_vertices = v_count;

    compute_vertex_shader->activate();
    for (auto i = 0; i < 2; ++i)
    {
        glBindVertexArray(vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);
        glEnableVertexAttribArray(0);   // position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)0);
        glEnableVertexAttribArray(1);   // velocity
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(3*sizeof(float)));
        glEnableVertexAttribArray(2);   // color
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(6*sizeof(float)));
        glEnableVertexAttribArray(3);   // lifetime
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(10*sizeof(float)));
        glEnableVertexAttribArray(4);   // life
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(13*sizeof(float)));

        glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, tfbo[1-i]);
        glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbo[i]);
    }
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    draw_shader->activate();
    draw_shader->set("use_texture", 0);
    draw_shader->set("use_atlas", 0);
    glBindVertexArray(vao[2]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);  // vertices
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*n_vertices, vertex, GL_STATIC_DRAW);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);  // scale + rotation
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    draw_shader->activate(false);

}

void scene::BenchmarkScene::TransformFeedbackParticleSystem::restart()
{
    debt_particles = 0.f;
    _tot_particles = max_particles;

    need_upload = true;
}

void scene::BenchmarkScene::TransformFeedbackParticleSystem::upload()
{
    if (!need_upload) return;

    auto s  = sizeof(float)*17*max_particles;

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, s, nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, s, nullptr, GL_DYNAMIC_DRAW);

    auto scal_rot = new float[2*max_particles];
#pragma omp parallel for num_threads(8)
    for (decltype(max_particles) i = 0; i < max_particles; ++i)
    {
        scal_rot[i] = i % 2 == 0 ? 0.025f : 0.f;
    }
    glBindVertexArray(vao[2]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*max_particles, scal_rot, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    delete [] scal_rot;

    n_particles = max_particles;
    update(-1.f);
    n_particles = 0;
    need_upload = false;
}

void scene::BenchmarkScene::TransformFeedbackParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    compute_vertex_shader->activate();
    compute_vertex_shader->set("dt", dt);

    glEnable(GL_RASTERIZER_DISCARD);

    glBindVertexArray(vao[0]);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, tfbo[0]);
    glBeginTransformFeedback(GL_POINTS);
    glDrawArrays(GL_POINTS, 0, count());
    glEndTransformFeedback();
    glFlush();

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
    glBindVertexArray(0);
    glDisable(GL_RASTERIZER_DISCARD);
    compute_vertex_shader->activate(false);

    std::swap(vao[0], vao[1]);
    std::swap(vbo[0], vbo[1]);
    std::swap(tfbo[0], tfbo[1]);

    if (n_particles != total())
    {
        debt_particles += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(debt_particles);
        debt_particles -= new_particles;
        n_particles += new_particles;
        if (n_particles > total())
            n_particles = total();
    }
}

void scene::BenchmarkScene::TransformFeedbackParticleSystem::render(GLenum gl_draw_mode)
{
    draw_shader->activate();
    glBindVertexArray(vao[2]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(14*sizeof(float)));
    glEnableVertexAttribArray(2);   // color
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float)*17, (void *)(6*sizeof(float)));

    glVertexAttribDivisor(5, 0);    // vertices
    glVertexAttribDivisor(0, 1);    // position
    glVertexAttribDivisor(2, 1);    // color
    glDrawArraysInstanced(gl_draw_mode, 0, n_vertices, count());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    draw_shader->activate(false);
}

#endif // PX_CG_SCENE_IMPL_BENCHMARK_TRANSFORM_FEEDBACK_PARTICLE_SYSTEM
