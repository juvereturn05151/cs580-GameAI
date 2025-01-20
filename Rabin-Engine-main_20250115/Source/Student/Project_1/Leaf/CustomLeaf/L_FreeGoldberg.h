#pragma once
#include "BehaviorNode.h"
class L_FreeGoldberg : public BaseNode <L_FreeGoldberg>
{

public:
	L_FreeGoldberg();

protected:
	float timer;
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
};

