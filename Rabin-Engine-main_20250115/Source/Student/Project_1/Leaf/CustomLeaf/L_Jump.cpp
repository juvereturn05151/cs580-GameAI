#include "pch.h"
#include "L_Jump.h"

L_Jump::L_Jump() : timer(0.0f)
{}

void L_Jump::on_enter()
{
    // Assign a random jump duration (if needed)
    timer = RNG::range(1.0f, 2.0f);

    // Trigger the jump action once
    agent->jump(timer);

    BehaviorNode::on_leaf_enter();
}

void L_Jump::on_update(float dt)
{
    // Decrement the timer
    timer -= dt;

    // Check if the jump is complete
    if (timer < 0.0f && agent->is_grounded())
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}