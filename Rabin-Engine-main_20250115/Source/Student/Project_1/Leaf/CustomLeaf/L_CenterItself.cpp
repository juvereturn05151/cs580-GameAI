#include "pch.h"
#include "L_CenterItself.h"

L_CenterItself::L_CenterItself()
{}

void L_CenterItself::on_enter()
{
    // Trigger the jump action once
    agent->set_pitch(0.0f);
    agent->set_yaw(-1.5f);
    agent->set_roll(0.0f);
    on_success();
    BehaviorNode::on_leaf_enter();
}

void L_CenterItself::on_update(float dt)
{
    // Optional: log or display debugging info
    display_leaf_text();
}