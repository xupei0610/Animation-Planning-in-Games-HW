#include <fstream>
#include "scene/smoke_particle_scene.hpp"
#include "scene.hpp"
#include "util/random.hpp"
#include "stb_image.hpp"
#include "global.hpp"
#include "app.hpp"

using namespace px;

#define WORK_GROUP_SIZE 256

scene::SmokeParticleScene::SmokeParticleSystem::SmokeParticleSystem()
    : ParticleSystem(),
      vao{0}, vbo{0}, tfbo{0}, texture{0},
      compute_shader(nullptr), draw_shader(nullptr)
{}

scene::SmokeParticleScene::SmokeParticleSystem::~SmokeParticleSystem()
{
    glDeleteVertexArrays(2, vao);
    glDeleteBuffers(2, vbo);
    glDeleteTextures(2, texture);
    glDeleteTransformFeedbacks(2, tfbo);
    delete compute_shader;
    delete draw_shader;
}

void scene::SmokeParticleScene::SmokeParticleSystem::init(float *vertex,
                                                          unsigned int v_count,
                                                          unsigned int tex,
                                                          float *uv, bool atlas)
{


    const static char C_VS[] = "#version 420 core\n"
            "layout (location = 0) in vec3 position;"
            "layout (location = 1) in float life;"
            ""
            "out Particle"
            "{"
            "   vec3 position;"
            "   float life;"
            "} particle;"
            ""
            "out Pos"
            "{"
            "   vec4 ld;"
            "   vec4 rd;"
            "   vec4 lu;"
            "   vec4 ru;"
            "} gPosition;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "uniform sampler3D velocity_sampler;"
            "uniform float dt;"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn(vec2 sd)"
            "{"
            "   float theta = PI * (rnd(sd) * 2.f - 1.f); ++sd.x;"
            "   float r0 = rnd(sd) * .25f; ++sd.x;"
            "   particle.position = vec3(r0 * cos(theta),"
            "                            rnd(sd) * .1f - 3.f,"
            "                            r0 * sin(theta));"
            "   particle.life = 1.f;"
            "}"
            ""
            "void main()"
            "{"
            "   if (dt < 0.f || particle.position.y > 3.f)"
            "   {"
            "       spawn(vec2(gl_VertexID, gl_VertexID));"
            "   }"
            "   else"
            "   {"
            "       vec2 sd = particle.position.xy;"
            "       particle.life -= dt * 2.5f;"
            "       if (particle.life <= 0.f)"
            "           spawn(sd);"
            "   }"
            ""
            "   vec3 velocity = texture(velocity_sampler,"
            "                       vec3((particle.position.x + 2.f)/4.f,"
            "                            (particle.position.y + 4.f)/8.f,"
            "                            (particle.position.z + 4.f)/4.f)"
            "                   ).rgb;"
            "   vec3 mid_position = particle.position.xyz + velocity * (.5f * dt);"
            "   velocity = texture(velocity_sampler,"
            "                       vec3((mid_position.x + 2.f)/4.f,"
            "                            (mid_position.y + 4.f)/8.f,"
            "                            (mid_position.z + 4.f)/4.f)"
            "                   ).rgb;"
            "   particle.position += velocity * dt;"
            ""
            "   float size_y = .03f;"
            "   "
            "   float t;"
            "   vec3 u = mat3(view) * normalize(velocity);"    // velocity direction in view coordinate
            "   float nz = abs(normalize(u).z);"
            "   if (nz > 1.f - 1e-7f)"              // velocity along z, ignore y
            "       t = 1.f + (nz - 1.f) / 1e-7f;"
            "   else if (dot(u, u) < 1e-7f)"
            "       t = 1.f - dot(u, u) / 1e-7f;"  // velocity is too small, ignore velocity, horizon and vertical size is close
            "   else"
            "       t = 0.f;"
            ""
            "   float size_x = mix(5.f*size_y, size_y, t);"
            ""
            "   u.z = 0.f;"
            "   u = normalize(mix(normalize(u), vec3(1.f, 0.f, 0.f), t));"
            "   vec3 v = vec3(-u.y, u.x, 0.f);"
            "   vec3 a = u * mat3(view);"
            "   vec3 b = v * mat3(view);"
            "   mat3 scale = mat3(a, b, cross(a, b));"
            ""
//            "   vec3 up = scale * vec3(0.f, -size_y, 0.f);"
//            "   vec3 dn = scale * vec3(0.f,  size_y, 0.f);"
//            "   vec3 lf = scale * vec3( size_x, 0.f, 0.f);"
//            "   vec3 rg = scale * vec3(-size_x, 0.f, 0.f);"
            ""
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]);"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]);"
            "	vec3 dn = up_vec * (-size_y);"
            "	vec3 up = up_vec * (size_y);"
            "	vec3 lf = right_vec * (-size_x);"
            "	vec3 rg = right_vec * (size_x);"
            ""
            "   mat4 VP = proj * view;"
            "   gPosition.ld = VP * vec4(particle.position + dn + lf, 1.f);"
            "   gPosition.rd = VP * vec4(particle.position + dn + rg, 1.f);"
            "   gPosition.lu = VP * vec4(particle.position + up + lf, 1.f);"
            "   gPosition.ru = VP * vec4(particle.position + up + rg, 1.f);"
            ""
            "   particle.life = velocity.x;"
            "}";

    const static char VS[] = "#version 330 core\n"
            ""
            "layout (location = 0) in float alpha;"
            "layout (location = 1) in vec4 gPosition_ld;"
            "layout (location = 2) in vec4 gPosition_rd;"
            "layout (location = 3) in vec4 gPosition_lu;"
            "layout (location = 4) in vec4 gPosition_ru;"
            ""
            "out VS_OUT"
            "{"
            "       float alpha;"
            "       vec4 gPosition_ld;"
            "       vec4 gPosition_rd;"
            "       vec4 gPosition_lu;"
            "       vec4 gPosition_ru;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "   primitive.alpha = alpha;"
            "   primitive.gPosition_ld = gPosition_ld;"
            "   primitive.gPosition_rd = gPosition_rd;"
            "   primitive.gPosition_lu = gPosition_lu;"
            "   primitive.gPosition_ru = gPosition_ru;"
            "}";

    const static char GS[] = "#version 330 core\n"
            "layout(points) in;"
            "layout(triangle_strip, max_vertices = 4) out;"
            ""
            "in VS_OUT"
            "{"
            "       float alpha;"
            "       vec4 gPosition_ld;"
            "       vec4 gPosition_rd;"
            "       vec4 gPosition_lu;"
            "       vec4 gPosition_ru;"
            "} primitive[];"
            ""
            "out vec2  gTexCoord;"
            "out float gAlpha;"
            "void main()"
            "{"
            "   gAlpha = primitive[0].alpha;"
            ""
            "   gTexCoord = vec2(0, 0);"
            "   gl_Position = primitive[0].gPosition_ld;"
            "   EmitVertex();"
            "   gTexCoord = vec2(1, 0);"
            "   gl_Position = primitive[0].gPosition_rd;"
            "   EmitVertex();"
            "   gTexCoord = vec2(0, 1);"
            "   gl_Position = primitive[0].gPosition_lu;"
            "   EmitVertex();"
            "   gTexCoord = vec2(1, 1);"
            "   gl_Position = primitive[0].gPosition_ru;"
            "   EmitVertex();"
            ""
            "   EndPrimitive();"
            "}";

    const static char FS[] = "#version 330 core\n"
            "in vec2 gTexCoord;"
            "in float gAlpha;"
            ""
            "out vec4 color;"
            ""
            "uniform sampler2D alpha_sampler;"
            ""
            "void main()"
            "{"
            "   float falloff = texture(alpha_sampler, gTexCoord).r;"
            "   color = vec4(0.f, .05f, .1f, .85f * gAlpha * falloff);"
            "}";

    if (compute_shader == nullptr)
    {
        static const char *feedback[] = {
                "Particle.position",
                "Particle.life",
                "Pos.ld",
                "Pos.rd",
                "Pos.lu",
                "Pos.ru"
        };
        compute_shader = new Shader(C_VS, feedback, 6);
    }

    if (draw_shader == nullptr)
    {
        draw_shader = new Shader(VS, FS, GS);
    }


    glDeleteVertexArrays(3, vao);
    glDeleteBuffers(2, vbo);
    glDeleteTextures(2, texture);
    glDeleteTransformFeedbacks(2, tfbo);

    glGenVertexArrays(3, vao);
    glGenBuffers(2, vbo);
    glGenTextures(2, texture);
    glGenTextures(2, tfbo);

    compute_shader->activate();
    compute_shader->bind("GlobalAttributes", 0);
    compute_shader->set("velocity_sampler", 0);
    for (auto i = 0; i < 2; ++i)
    {
        glBindVertexArray(vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);

        glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, tfbo[1-i]);
        glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbo[i]);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    draw_shader->activate();
    draw_shader->set("alpha_sampler", 0);

    draw_shader->activate(false);

    // advect
    constexpr auto sphere_center_x = 0.f;
    constexpr auto sphere_center_y = 0.f;
    constexpr auto sphere_center_z = 0.f;
    constexpr auto sphere_radius = 0.f;
    constexpr float noise_scale[] = {0.4f, 0.23f, 0.11f};
    constexpr float noise_gain[] = {1.0f, 0.5f, 0.25f};
    constexpr auto field_height = -3.f;
    constexpr auto top_height = 8.f;
    constexpr auto ceiling_height = 3.f;
    constexpr auto ring_radius = 1.25f;
    constexpr auto ring_mag = 10.f;
    constexpr auto ring_falloff = .7f;
    constexpr auto ring_speed = 2.4f;
    unsigned int seed = std::random_device{}();
    constexpr int n = 128;
    auto ramp = [&](float v)
    {
        return v < 0 ? 0 : v > 1 ? 1 : v*v*v*(10 + v*(v*6-15));
    };
    auto lerp = [&](float v0, float v1, float t)
    {
        return (1 - t) * v0 + v1;
    };
    auto bilerp = [&](float v00, float v10,
                     float v01, float v11,
                     float t0, float t1)
    {
        return lerp(lerp(v00, v10, t0), lerp(v01, v11, t0), t1);
    };
    auto trilerp = [&](float v000, float v100,
                      float v010, float v110,
                      float v001, float v101,
                      float v011, float v111,
                      float t0, float t1, float t2)
    {
        return lerp(bilerp(v000, v100, v010, v110, t0, t1),
                    bilerp(v001, v101, v011, v111, t0, t1),
                    t2);
    };
    auto randhash = [&](int seed)
    {
        unsigned int i=(seed^12345391u)*2654435769u;
        i^=(i<<6)^(i>>26);
        i*=2654435769u;
        i+=(i<<5)^(i>>12);
        return i;
    };
    unsigned int perm[n];
    glm::vec3 basis[n];
    for (auto i = 0; i < n; ++i)
    {
        for (auto j = 0; j < 3; ++j)
            basis[i][j] = rnd_np();
        basis[i] /= std::sqrt(basis[i][0]*basis[i][0] + basis[i][1]*basis[i][1] + basis[i][2]*basis[i][2]);
        perm[i] = i;
    }
    for (auto i = 1; i < n; ++i)
    {
        std::swap(perm[i], perm[randhash(seed++)%(i+1)]);
    }
    auto hash_index = [&](unsigned int x, unsigned int y, unsigned int z)
    {
        return perm[(perm[(perm[x%n]+y)%n]+z)%n];
    };
    auto noise = [&](float x, float y, float z)
    {
        auto &v000 = basis[hash_index(int(x),  int(y),  int(z))];
        auto &v100 = basis[hash_index(int(x)+1,int(y),  int(z))];
        auto &v010 = basis[hash_index(int(x),  int(y)+1,int(z))];
        auto &v110 = basis[hash_index(int(x)+1,int(y)+1,int(z))];
        auto &v001 = basis[hash_index(int(x),  int(y),  int(z)+1)];
        auto &v101 = basis[hash_index(int(x)+1,int(y),  int(z)+1)];
        auto &v011 = basis[hash_index(int(x),  int(y)+1,int(z)+1)];
        auto &v111 = basis[hash_index(int(x)+1,int(y)+1,int(z)+1)];

        auto fx = x - int(x); auto fy = y - int(y); auto fz = z - int(z);
        auto sx = fx*fx*fx*(10 + fx*(fx*6-15));
        auto sy = fy*fy*fy*(10 + fy*(fy*6-15));
        auto sz = fz*fz*fz*(10 + fz*(fz*6-15));
        return trilerp(   fx  * v000[0] +    fy  * v000[1] +    fz  * v000[2],
                       (fx-1) * v100[0] +    fy  * v100[1] +    fz  * v100[2],
                          fx  * v010[0] + (fy-1) * v010[1] +    fz  * v010[2],
                       (fx-1) * v110[0] + (fy-1) * v110[1] +    fz  * v110[2],
                          fx  * v001[0] +    fy  * v001[1] + (fz-1) * v001[2],
                       (fx-1) * v101[0] +    fy  * v101[1] + (fz-1) * v101[2],
                          fx  * v011[0] + (fy-1) * v011[1] + (fz-1) * v011[2],
                       (fx-1) * v111[0] + (fy-1) * v111[1] + (fz-1) * v111[2],
                       sx, sy, sz);
    };

    auto dist = [&](float x, float y, float z)
    {
        x -= sphere_center_x;
        y -= sphere_center_y;
        z -= sphere_center_z;
        return std::sqrt(x*x + y*y + z*z) - sphere_radius;
    };
    auto grad = [&](glm::vec3 &p)
    {
        constexpr auto eps = .01f;

        auto d = dist(p.x, p.y, p.z);
        auto dx = dist(p.x + eps, p.y, p.z) - d;
        auto dy = dist(p.x, p.y + eps, p.z) - d;
        auto dz = dist(p.x, p.y, p.z + eps) - d;
        auto l2 = std::sqrt(dx*dx + dy*dy + dz*dz);

        p.x = dx / l2;
        p.y = dy / l2;
        p.z = dz / l2;
    };
    auto potential = [&](float x, float y, float z, int id)
    {
        auto distance = dist(x, y, z);
        glm::vec3 gradient(x, y, z);
        grad(gradient);

        glm::vec3 psi(0.f);

        auto hf = ramp((y - field_height) / top_height);
        for (auto i = 0; i < 3; ++i)
        {
            auto px = x / noise_scale[i];
            auto py = y / noise_scale[i];
            auto pz = z / noise_scale[i];
            auto t = ramp(std::fabs(distance) / noise_scale[i]);

            glm::vec3 p(noise(px, py, pz),
                        noise(py +  31.416f, pz -  47.853f, px + 12.793f),
                        noise(pz - 233.145f, px - 113.408f, py - 185.31f));

            psi += hf * noise_gain[i] * (t * p + (1 - t) * glm::dot(p, gradient) * gradient);
        }

        glm::vec3 force(y, 0.f, -x);
        float ring_y = ceiling_height;
        float t = ramp(std::fabs(distance) / ring_radius);
        while (ring_y > field_height)
        {
            auto r = std::sqrt(x*x + z*z);
            auto falloff = (r - ring_radius)*(r - ring_radius) + (r + ring_radius)*(r + ring_radius) + (y - ring_y)*(y - ring_y) + ring_falloff;
            auto mag = ring_mag / falloff;
            psi += t * mag * force + (1 - t) * glm::dot(force, gradient);
            ring_y -= ring_speed;
        }

        return psi[id];
    };
    auto curl = [&](glm::vec3 &p)
    {
        constexpr auto eps = 1e-4f;
        constexpr auto double_eps = 2*eps;

        p.x =  potential(p.x, p.y+eps, p.z, 2) - potential(p.x, p.y-eps, p.z, 2)
             - potential(p.x, p.y, p.z+eps, 1) + potential(p.x, p.y, p.z-eps, 1);
        p.y =  potential(p.x, p.y, p.z+eps, 0) - potential(p.x, p.y, p.z-eps, 0)
             - potential(p.x+eps, p.y, p.z, 2) + potential(p.x-eps, p.y, p.z, 2);
        p.z =  potential(p.x+eps, p.y, p.z, 1) - potential(p.x-eps, p.y, p.z, 1)
             - potential(p.x, p.y+eps, p.z, 0) + potential(p.x, p.y-eps, p.z, 0);

        p.x /= double_eps;
        p.y /= double_eps;
        p.z /= double_eps;
    };
    constexpr auto cols = 128; constexpr auto rows = 256; constexpr auto depth = 128;
    std::vector<float> curl_noise_based_velocity(3*cols*rows*depth, 0.f);
//    curl_noise_based_velocity.reserve(3*cols*rows*depth);
//    for (auto k = 0; k < depth; ++k)
//    {
//        for (auto i = 0; i < cols; ++i)
//        {
//            for (auto j = 0; j < rows; ++j)
//            {
//                glm::vec3 p(-2.f + 4.f * i / cols,
//                            (-2.f + 4.f * j / rows)  * rows / cols,
//                            (-2.f + 4.f * k / depth) * depth / cols);
//                curl(p);
//                curl_noise_based_velocity.push_back(p.x);
//                curl_noise_based_velocity.push_back(p.y);
//                curl_noise_based_velocity.push_back(p.z);
////                std::cout << p.x << " " << p.y << " " << p.z << std::endl;
//            }
//        }
//    }
//    std::ofstream fout("data.dat", std::ios::out | std::ios::binary);
//    fout.write((char *)&curl_noise_based_velocity[0], curl_noise_based_velocity.size()*sizeof(float));
//    fout.close();


    glBindTexture(GL_TEXTURE_3D, texture[0]);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB16F, cols, rows, depth, 0, GL_RGB, GL_FLOAT, curl_noise_based_velocity.data());
    glBindTexture(GL_TEXTURE_3D, 0);

    int w, h, ch;
    auto ptr = stbi_load(ASSET_PATH "/texture/particle3.png", &w, &h, &ch, 3);
    TEXTURE_LOAD_HELPER(texture[1], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
    stbi_image_free(ptr);
}

void scene::SmokeParticleScene::SmokeParticleSystem::restart()
{
    max_particles = 100000;
    birth_rate = 999999999;

    _tot_particles = max_particles;

    need_upload = true;
}

void scene::SmokeParticleScene::SmokeParticleSystem::upload()
{
    if (!need_upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, 20*sizeof(float)*total(), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, 20*sizeof(float)*total(), nullptr, GL_DYNAMIC_DRAW);

    n_particles = total();
    update(-1.f);

    need_upload = false;
}

void scene::SmokeParticleScene::SmokeParticleSystem::update(float dt,
                                                            glm::vec3 *cam_pos)
{
    glEnable(GL_RASTERIZER_DISCARD);

    compute_shader->activate();
    compute_shader->set("dt", dt);
    glBindVertexArray(vao[0]);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, tfbo[0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, texture[0]);

    glBeginTransformFeedback(GL_POINTS);
    glDrawArrays(GL_POINTS, 0, count());
    glEndTransformFeedback();
    glFlush();

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
    glBindTexture(GL_TEXTURE_3D, 0);
    glBindVertexArray(0);
    glDisable(GL_RASTERIZER_DISCARD);
    compute_shader->activate(false);

    std::swap(vao[0], vao[1]);
    std::swap(vbo[0], vbo[1]);
    std::swap(tfbo[0], tfbo[1]);
}

void scene::SmokeParticleScene::SmokeParticleSystem::render(GLenum gl_draw_mode)
{
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA);

    draw_shader->activate();
    glBindVertexArray(vao[2]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*20, (void *)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(float)*20, (void *)(4*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float)*20, (void *)(8*sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(float)*20, (void *)(12*sizeof(float)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(float)*20, (void *)(16*sizeof(float)));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    glDrawArrays(GL_POINTS, 0, count());

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    draw_shader->activate(false);
    glDisable(GL_BLEND);
}

scene::SmokeParticleScene::SmokeParticleScene()
    : BaseScene(),
      text(nullptr), particle_system(new SmokeParticleSystem)
{}

scene::SmokeParticleScene::~SmokeParticleScene()
{
    delete text;
    delete particle_system;
}

void scene::SmokeParticleScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);

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

void scene::SmokeParticleScene::restart(Scene &scene)
{
    scene.character.reset(0.f, scene.character.characterHeight(), 20.f, 0.f, 0.f);
    scene.character.setShootable(false);

    particle_system->restart();
}

void scene::SmokeParticleScene::upload(Shader &scene_shader)
{
    glClearColor(1.f, 1.f, 1.f, 1.f);
    particle_system->upload();
}

void scene::SmokeParticleScene::render(Shader &scene_shader)
{}

void scene::SmokeParticleScene::update(float dt)
{
    particle_system->update(dt);
    processInput(dt);
}

void scene::SmokeParticleScene::render()
{
    particle_system->render(GL_POINTS);

    renderInfo();
}

void scene::SmokeParticleScene::renderInfo()
{
    text->render("Particles: " + std::to_string(particle_system->count()) + "/" + std::to_string(particle_system->total()),
                 10, 10, .4f,
                 glm::vec4(0.0f, 0.0f, 0.0f, 1.0f),
                 Anchor::LeftTop);
    text->render("Rendering Mode: compute + geometry shader",
                 10, 30, .4f,
                 glm::vec4(0.0f, 0.0f, 0.0f, 1.0f),
                 Anchor::LeftTop);
}

void scene::SmokeParticleScene::processInput(float dt)
{}
