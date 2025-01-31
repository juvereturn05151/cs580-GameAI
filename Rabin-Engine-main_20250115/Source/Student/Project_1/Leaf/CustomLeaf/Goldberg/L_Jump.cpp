#include "pch.h"
#include "L_Jump.h"

L_Jump::L_Jump()
{}

void L_Jump::on_enter()
{
    // Trigger the jump action once
    agent->jump();

    BehaviorNode::on_leaf_enter();
}

void L_Jump::on_update(float dt)
{
    //printf("jump\n");
    // Check if the jump is complete
    if (agent->is_ground())
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}