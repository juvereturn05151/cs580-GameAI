#include "pch.h"
#include "L_FireUpward.h"

L_FireUpward::L_FireUpward()
{}

void L_FireUpward::on_enter()
{
    // Trigger the jump action once
    agent->fireUpward();

    BehaviorNode::on_leaf_enter();
}

void L_FireUpward::on_update(float dt)
{
    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}