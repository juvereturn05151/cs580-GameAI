#include "pch.h"
#include "L_Wander.h"

L_Wander::L_Wander()
{}

void L_Wander::on_enter()
{
    timer = 1.0f;

    BehaviorNode::on_leaf_enter();
}

void L_Wander::on_update(float dt)
{
    agent->wander();
    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }
    // Optional: log or display debugging info
    display_leaf_text();
}