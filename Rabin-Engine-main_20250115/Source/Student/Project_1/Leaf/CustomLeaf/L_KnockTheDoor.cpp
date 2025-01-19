#include "pch.h"
#include "L_KnockTheDoor.h"

L_KnockTheDoor::L_KnockTheDoor() : timer(0), knockState(0), knockTimer(0.0f), knockDuration(4.0f)
{}

void L_KnockTheDoor::on_enter()
{
    timer = 2.0f;
    knockTimer = 0.0f;             // Reset the knocking timer
    knockState = 1;
    original_pitch = agent->get_pitch();
    BehaviorNode::on_leaf_enter();
}

void L_KnockTheDoor::on_update(float dt)
{
    timer -= dt;

    // Perform knocking animation
    knockTimer += dt;
    if (knockTimer < knockDuration)
    {
        // Forward motion (simulate a knock)
        agent->set_pitch(original_pitch + (45.0f * knockState)); // Example: Add pitch for the knock
    }
    else if (knockTimer < 2 * knockDuration)
    {
        // Backward motion (reset position)
        agent->set_pitch(original_pitch + (45.0f * -knockState)); // Reverse the motion
    }
    else
    {
        // Reset the animation cycle
        knockTimer = 0.0f;
        knockState = -knockState; // Alternate knock direction (if needed)
    }

    if (timer < 0.0f)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}

void L_KnockTheDoor::on_exit()
{
    // Reset agent's pitch/yaw/roll to neutral position
    agent->set_pitch(original_pitch);
}