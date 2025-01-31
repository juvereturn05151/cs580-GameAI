#pragma once
#include "BehaviorNode.h"
class L_LeftPunch : public BaseNode<L_LeftPunch>
{
public:
    L_LeftPunch();

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    virtual void on_exit();
    Animation createSampleAnimation();
private:
    float original_yaw;
};

