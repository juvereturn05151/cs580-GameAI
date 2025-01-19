#include "pch.h"
#include "L_KnockTheDoor.h"


Animation L_KnockTheDoor::createSampleAnimation() {
    Animation anim;
    anim.duration = 1.0f; // 5 seconds

    Vec3 original_pos = agent->get_position();
    Vec3 original_scale = agent->get_scaling();
    float rotX = agent->get_pitch();
    float rotY = agent->get_yaw();
    float rotZ = agent->get_roll();

    anim.keyframes = {
        {original_pos, Vec3(rotX, rotY, rotZ), original_scale},
        { original_pos, Vec3(45.0f + rotX, rotY, rotZ), original_scale},
        { original_pos, Vec3(-45.0f + rotX, rotY, rotZ), original_scale}
    };

    return anim;
}

L_KnockTheDoor::L_KnockTheDoor() : timer(0), knockState(0), knockTimer(0.0f), knockDuration(4.0f)
{}

void L_KnockTheDoor::on_enter()
{
    timer = 2.0f;
    knockTimer = 0.0f;             // Reset the knocking timer
    knockState = 1;
    original_pitch = agent->get_pitch();

    agent->setAnimAndPlay(createSampleAnimation());

    BehaviorNode::on_leaf_enter();
}

void L_KnockTheDoor::on_update(float dt)
{
    timer -= dt;

    if (timer < 0.0f)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}

void L_KnockTheDoor::on_exit()
{
    agent->stopAnim();
    // Reset agent's pitch/yaw/roll to neutral position
    agent->set_pitch(original_pitch);
}