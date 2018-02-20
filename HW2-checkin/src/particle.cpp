#include "particle.hpp"

using namespace px;

Particle_t::Particle_t()
    : rotation(0.f), u_offset(.5f), v_offset(.5f), texture_scale(.5f)
{}

ParticleSystem::ParticleSystem()
{}

[ [noreturn] ]
void ParticleSystem::err(std::string const &msg)
{
    throw new ParticleSystemError("Particle System Error: " + msg);
}
