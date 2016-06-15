#include "LayoutComponent.h"

//LayoutComponent::LayoutComponent()
//{
//}

///**
// * @brief LayoutComponent::getParticle
// * @return particle, the parent particle pointer
// *
// * refs #523
// */
//Particle *LayoutComponent::getParticle() const
//{
//    return particle;
//}
//
///**
// * @brief LayoutComponent::setParticle
// * @param value sets the particle parent pointer
// *
// * refs #523
// */
//void LayoutComponent::setParticle(Particle *value)
//{
//    particle = value;
//}



Particle* LayoutComponent::getParticlePtr() const
{
    return particlePtr;
}

void LayoutComponent::setParticlePtr(Particle* value)
{
    particlePtr = value;
}
