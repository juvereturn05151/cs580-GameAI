#pragma once
#include "BehaviorNode.h"
class L_CheckIfGoldbergIsFree : public BaseNode<L_CheckIfGoldbergIsFree>
{
protected:
	virtual void on_update(float dt);
};

