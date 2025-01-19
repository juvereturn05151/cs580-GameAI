#include "pch.h"
#include "L_KnockTheDoor.h"


Animation L_KnockTheDoor::createSampleAnimation() {
    Animation anim;
    anim.frameSpeed = 1.0f; // 5 seconds

    Vec3 original_pos = agent->get_position();
    Vec3 original_scale = agent->get_scaling();
    float rotX = agent->get_pitch();
    float rotY = agent->get_yaw();
    float rotZ = agent->get_roll();

    anim.keyframes = {
         {original_pos, Vec3(rotX, rotY, rotZ), original_scale},
        { original_pos, Vec3(0.40f, rotY, rotZ), original_scale},
        { original_pos, Vec3(rotX, rotY, rotZ), original_scale},
    };

    return anim;
}

L_KnockTheDoor::L_KnockTheDoor()
{}

void L_KnockTheDoor::on_enter()
{
    original_pitch = agent->get_pitch();

    agent->setAnimAndPlay(createSampleAnimation());

    BehaviorNode::on_leaf_enter();
}

void L_KnockTheDoor::on_update(float dt)
{
    if (agent->getAnimationComp()->getCurrentFrame() == 2)
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