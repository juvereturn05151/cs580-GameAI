#include "pch.h"
#include "PhysicsComponent.h"
#include "Agent/Agent.h"

#define GRAVITY -9.81
#define GROUND_HEIGHT 0.0
#define SKY_HEIGHT 1000.0

PhysicsComponent::PhysicsComponent(Agent& _owner, float _mass) : Component(_owner), mass(_mass)
{
	velocity = Vec3(0, 0, 0);
}

void PhysicsComponent::update(float deltaTime)
{
	printf("pos Y: %f\n", owner->get_position().y);
	//printf("velocity Y: %f\n", velocity.y);*/

	if (!isActive)
	{
		return;
	}

	Vec3 gravity(0.0f, GRAVITY * mass, 0.0f);
	accumulatedForce += gravity;

	Vec3 acceleration = accumulatedForce / mass;
	velocity += acceleration * deltaTime;

	// Apply damping to simulate friction, reducing velocity over time
	// Adjust based on desired friction level
	velocity *= dampingFactor;

	Vec3 pos = owner->get_position();
	pos.x += velocity.x * deltaTime;
	pos.y += velocity.y * deltaTime;
	pos.z += velocity.z * deltaTime;



	// Clamp the y position to ensure it doesn't go below 0
	if (pos.y < GROUND_HEIGHT)
	{
		is_ground = true;
		pos.y = GROUND_HEIGHT;
		velocity.y = 0.0f; // Reset the y velocity to prevent bouncing below the ground
	}

	owner->set_position(pos);

	// Reset accumulated force for next frame
	accumulatedForce = Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsComponent::applyForce(const Vec3& force)
{
	accumulatedForce = accumulatedForce + force;

	if (accumulatedForce.y > 0)
	{
		is_ground = false;
	}
}

void PhysicsComponent::setVelocity(const Vec3& velocity)
{
	this->velocity = velocity;
}

Vec3 PhysicsComponent::getVelocity() const
{
	return velocity;
}

float PhysicsComponent::getMass() const
{
	return mass;
}

bool PhysicsComponent::isGround()
{
	printf("owner->get_position().y: %f\n", owner->get_position().y);
	printf("GROUND_HEIGHT: %f\n\n", GROUND_HEIGHT);
	return is_ground;
}

bool PhysicsComponent::isTooHigh()
{
	return owner->get_position().y >= SKY_HEIGHT;
}
