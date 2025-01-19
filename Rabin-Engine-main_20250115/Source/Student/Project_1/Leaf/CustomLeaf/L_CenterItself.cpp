#include "pch.h"
#include "L_CenterItself.h"

L_CenterItself::L_CenterItself()
{}

void L_CenterItself::on_enter()
{
    // Trigger the jump action once
    timer = 0.5f;
    agent->set_pitch(0.0f);
    agent->set_yaw(-1.5f);
    agent->set_roll(0.0f);
    printf("L_CenterItself\n");
    BehaviorNode::on_leaf_enter();
}

void L_CenterItself::on_update(float dt)
{
    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }
    // Optional: log or display debugging info
    display_leaf_text();
}

void L_CenterItself::on_exit()
{
    printf("L_CenterItself exit\n");
}