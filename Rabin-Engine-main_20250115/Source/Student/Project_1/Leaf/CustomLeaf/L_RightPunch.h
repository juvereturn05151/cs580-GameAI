#pragma once
#include "BehaviorNode.h"
class L_RightPunch : public BaseNode<L_RightPunch>
{
public:
    L_RightPunch();

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit();
    Animation createSampleAnimation();
private:
    float original_yaw;
};

