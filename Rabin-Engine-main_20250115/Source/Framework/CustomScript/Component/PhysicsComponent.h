/*
Author: Ju-ve Chankasemporn
E-mail: juvereturn@gmail.com
Brief: Handles the physics behavior of a GameObject by managing forces, velocity, and updates based on the simulation's delta time.
*/

#pragma once

#ifndef PHYSICSCOMPONENT_H
#define PHYSICSCOMPONENT_H

#include "../Misc/NiceTypes.h" // For 3D vector math operations
#include "Component.h"  // Base class for all components

// The PhysicsComponent class handles physics-related calculations for a GameObject.
// This includes velocity updates, force accumulation, and damping.
class PhysicsComponent : public Component {
private:
    const float dampingFactor = 0.98f; // A constant factor to simulate friction or air resistance
    Vec3 velocity;                  // The current velocity of the GameObject
    float mass;                        // The mass of the GameObject, used for force calculations
    Vec3 accumulatedForce;          // The sum of all forces applied during a single frame
    bool is_ground;
    bool is_Flying;

public:
    // Constructor: Initializes the PhysicsComponent with an owner GameObject and its mass.
    PhysicsComponent(Agent& _owner, float mass);

    // Initializes the PhysicsComponent. Can be extended for more complex setups.
    void initialize() {};

    // Updates the PhysicsComponent every frame.
    // - Integrates forces to compute velocity and position changes.
    // - Applies damping to slow the object over time.
    void update(float deltaTime);

    // Applies a force to the object. This force is added to the accumulatedForce.
    // Forces should be cleared each frame after being applied.
    void applyForce(const Vec3& force);

    // Sets the velocity of the object directly.
    void setVelocity(const Vec3& velocity);

    // Gets the current velocity of the object.
    Vec3 getVelocity() const;

    // Gets the mass of the object.
    float getMass() const;

    bool isGround();

    bool isTooHigh();

    void setIsFlying(bool flying);

    void Separation();
    void Alignment();
    void Cohesion();
    void Seek(const Vec3& target);

    Vec3 normalized(Vec3 result);
    float DistanceSquared(Vec3 firstVec, Vec3 secondVec);
    float Distance(Vec3 firstVec, Vec3 secondVec);
};

#endif