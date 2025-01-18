/*
Author: Ju-ve Chankasemporn
E-mail: juvereturn@gmail.com
Brief: Component System that can be extendable
*/

#pragma once

#ifndef COMPONENT_H
#define COMPONENT_H

class Agent;

class Component
{
protected:
	Agent* owner;
	bool isActive;
public:
	Component(Agent& _owner);
	virtual void initialize() {};
	virtual void update(float deltaTime) {};
	virtual ~Component();

	void setIsActive(bool active);
};

#endif