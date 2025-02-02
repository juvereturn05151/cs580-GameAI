#include "pch.h"
#include "PhysicsComponent.h"
#include "Agent/Agent.h"
#include "../CustomScript/FlockController.h"

#define GRAVITY -20
#define GROUND_HEIGHT 0.0
#define SKY_HEIGHT 1000.0

PhysicsComponent::PhysicsComponent(Agent& _owner, float _mass) : Component(_owner), mass(_mass)
{
	velocity = Vec3(0, 0, 0);
	is_Flying = false;
}

void PhysicsComponent::update(float deltaTime)
{
	if (!isActive)
	{
		return;
	}

	Vec3 gravity(0.0f, GRAVITY * mass, 0.0f);

	if (is_Flying)
	{
		gravity = Vec3(0.0f, 0.0f, 0.0f);
	}

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

Vec3 PhysicsComponent::normalized(Vec3 result)
{
	float len = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
	return Vec3(result.x / len, result.y / len, result.z / len);
}

float PhysicsComponent::DistanceSquared(Vec3 firstVec, Vec3 secondVec)
{
	return (firstVec.x - secondVec.x) * (firstVec.x - secondVec.x) +
		(firstVec.y - secondVec.y) * (firstVec.y - secondVec.y) +
		(firstVec.z - secondVec.z) * (firstVec.z - secondVec.z);
}

float PhysicsComponent::Distance(Vec3 firstVec, Vec3 secondVec)
{
	return std::sqrt(DistanceSquared(firstVec, secondVec));
}

void PhysicsComponent::Separation()
{
	Vec3 steer;
	int count = 0;

	for (BehaviorAgent* other : FlockController::get_instance().boids) {
		if (other == owner) continue; // Skip self

		float distance = Distance(owner->get_position(), other->get_position());

		if (std::isnan(distance) || std::isinf(distance)) {
			//std::cout << "Invalid distance detected!" << std::endl;
			continue; // Skip invalid values
		}

		if (distance < FlockController::get_instance().perceptionRadius)
		{
			/*std::cout << "debug less than" << std::endl;
			std::cout << "owner x: " << owner->get_position().x << " owner y: " << owner->get_position().y << std::endl;
			std::cout << "other x: " << other->get_position().x << " other y: " << other->get_position().y << std::endl;*/

			Vec3 diff = owner->get_position() - other->get_position();
			diff = normalized(diff); // Apply safe normalization

			steer = steer + diff;
			count++;
		}
	}

	if (count > 0)
	{
		accumulatedForce += (steer * (1.0f / count)) * 1.5f;;
	}
	else
	{
		accumulatedForce += steer;
	}
}

void PhysicsComponent::Alignment()
{
	Vec3 avgVelocity;
	int count = 0;

	for (BehaviorAgent* other : FlockController::get_instance().boids) {
		if (other == owner) continue; // Skip self

		float distance = Distance(owner->get_position(), other->get_position());

		if (std::isnan(distance) || std::isinf(distance)) {
			continue; // Skip invalid values
		}

		if (distance < FlockController::get_instance().perceptionRadius)
		{
			avgVelocity = avgVelocity + other->getPhysicsComp()->velocity;
			count++;
		}
	}

	if (count > 0)
	{
		accumulatedForce += (avgVelocity * (1.0f / count));
	}
	else
	{
		accumulatedForce += avgVelocity;
	}
}

void PhysicsComponent::Cohesion()
{
	Vec3 steer;
	int count = 0;

	for (BehaviorAgent* other : FlockController::get_instance().boids) {
		if (other == owner) continue; // Skip self

		float distance = Distance(owner->get_position(), other->get_position());

		if (std::isnan(distance) || std::isinf(distance)) {
			//std::cout << "Invalid distance detected!" << std::endl;
			continue; // Skip invalid values
		}

		if (distance < FlockController::get_instance().perceptionRadius)
		{
			Vec3 diff = owner->get_position() + other->get_position();
			diff = normalized(diff); // Apply safe normalization

			steer = steer + diff;
			count++;
		}
	}

	if (count > 0)
	{
		accumulatedForce += (steer * (1.0f / count));
	}
	else
	{
		accumulatedForce += steer;
	}
}

void PhysicsComponent::Wander()
{
	accumulatedForce += Vec3((rand() % 3 - 1) * 0.5f,0, (rand() % 3 - 1) * 0.5f) * 500.0f;
}

void PhysicsComponent::Seek(const Vec3& target)
{
	Vec3 desired = normalized((target - owner->get_position())) * owner->get_movement_speed();
	accumulatedForce += normalized(desired - velocity) * 4000.0f;
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
	return is_ground;
}

bool PhysicsComponent::isTooHigh()
{
	return owner->get_position().y >= SKY_HEIGHT;
}

void PhysicsComponent::setIsFlying(bool flying)
{
	is_Flying = flying;
}