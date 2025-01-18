#include "pch.h"
#include "Component.h"
#include "Agent/Agent.h"

Component::Component(Agent& _owner) : owner(&_owner), isActive(true) {}


void Component::setIsActive(bool active)
{
	isActive = active;
}

Component::~Component()
{

}
