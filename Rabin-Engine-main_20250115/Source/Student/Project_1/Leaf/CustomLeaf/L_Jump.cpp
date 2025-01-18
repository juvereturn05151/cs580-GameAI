#include "pch.h"
#include "L_Jump.h"

L_Jump::L_Jump() : timer(0.0f)
{}

void L_Jump::on_enter()
{
    timer = RNG::range(1.0f, 2.0f);

    BehaviorNode::on_leaf_enter();
}

void L_Jump::on_update(float dt)
{
    agent->jump(timer);

    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }

    display_leaf_text();
}