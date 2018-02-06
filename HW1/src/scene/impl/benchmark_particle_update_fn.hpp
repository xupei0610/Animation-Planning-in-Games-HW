#ifndef PX_CG_SCENE_IMPL_BENCHMARK_PARTICLE_UPDATE_FN_HPP
#define PX_CG_SCENE_IMPL_BENCHMARK_PARTICLE_UPDATE_FN_HPP

const ParticleSpawnFn_t spawn_fn = [&](Particle_t &p, float &life, int i)
{
    auto theta = PI * rnd_np();
    auto phi =  std::acos(rnd_np());
    auto r0 = std::cbrt(rnd() * 125.f);
    pimpl->position[i].x = r0 * std::sin(phi) * std::cos(theta);
    pimpl->position[i].y = r0 * std::cos(phi);
    pimpl->position[i].z = r0 * std::sin(phi) * std::sin(theta);

    pimpl->velocity[i].x = 0.0001f * p.position.x;
    pimpl->velocity[i].y = 0.0001f * p.position.y;
    pimpl->velocity[i].z = 0.0001f * p.position.z;

    pimpl->acceleration[i].x = 0.f;
    pimpl->acceleration[i].y = 0.f;
    pimpl->acceleration[i].z = 0.f;

    pimpl->life[i].x = 4.f + (2*rnd() - 1.f) * (4.f / 4);
    pimpl->life[i].y = 1.f + (2*rnd() - 1.f) * (1.f / 4);
    pimpl->life[i].z = 5.f + (2*rnd() - 1.f) * (5.f / 4);

    life = pimpl->life[i].x + pimpl->life[i].y + pimpl->life[i].z;

    p.position = pimpl->position[i];

    p.color.r = 1.f;
    p.color.g = 0.45f;
    p.color.b = 0.2f;
    p.color.a = 0.f;

    p.size = 0.025f;
    p.rotation = 0.f;

    return true;
};

static const ParticleUpdateFn_t update_fn = [&](Particle_t &p, float &life, float dt, int i)
{
    if (life > 0.f)
        life -= dt;

    if (life <= 0.f)
        return false;

    if (life > pimpl->life[i].y + pimpl->life[i].z)
        p.color.a += dt / pimpl->life[i].x;
    else if (life > pimpl->life[i].z)
        p.color.a = 1.f;
    else
    {
        p.color.a -= dt / pimpl->life[i].z;

        pimpl->velocity[i].x += pimpl->acceleration[i].x + .01 * (2*rnd() - 1.f);
        pimpl->velocity[i].y += pimpl->acceleration[i].y + .01 * (2*rnd() - 1.f);
        pimpl->velocity[i].z += pimpl->acceleration[i].z + .01 * (2*rnd() - 1.f);

        pimpl->position[i] += pimpl->velocity[i];
    }
    p.color.a = std::min(1.f, std::max(0.f, p.color.a));

    pimpl->rotation[i] += drot * dt;
    if (pimpl->rotation[i] > PI) pimpl->rotation[i] -= double_PI;
    auto rot_c = std::cos(pimpl->rotation[i]);
    auto rot_s = std::sin(pimpl->rotation[i]);
    p.position.x = rot_c * pimpl->position[i].x + rot_s * pimpl->position[i].z;
    p.position.y = pimpl->position[i].y;
    p.position.z = rot_c * pimpl->position[i].z - rot_s * pimpl->position[i].x;

    return true;
};


#endif // PX_CG_SCENE_IMPL_BENCHMARK_PARTICLE_UPDATE_FN_HPP
