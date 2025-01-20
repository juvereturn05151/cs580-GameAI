#include "pch.h"
#include "L_AutoRotate.h"

L_AutoRotate::L_AutoRotate() : timer(0.0f)
{}

void L_AutoRotate::on_enter()
{
    timer = 2.0f;

    BehaviorNode::on_leaf_enter();
}

void L_AutoRotate::on_update(float dt)
{
    agent->set_yaw(agent->get_yaw() + 15.0f);

    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }

    display_leaf_text();
}
