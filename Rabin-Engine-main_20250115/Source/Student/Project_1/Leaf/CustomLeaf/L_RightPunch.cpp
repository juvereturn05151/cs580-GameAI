#include "pch.h"
#include "L_RightPunch.h"

Animation L_RightPunch::createSampleAnimation() {
    Animation anim;
    anim.frameSpeed = 1.0f; // 5 seconds

    Vec3 original_pos = agent->get_position();
    Vec3 original_scale = agent->get_scaling();
    float rotX = agent->get_pitch();
    float rotY = agent->get_yaw();
    float rotZ = agent->get_roll();

    anim.keyframes = {
         {original_pos, Vec3(rotX, rotY, rotZ), original_scale},
        { original_pos, Vec3(rotX, -2.0f, rotZ), original_scale},
        { original_pos, Vec3(rotX, rotY, rotZ), original_scale},
    };

    return anim;
}

L_RightPunch::L_RightPunch()
{}

void L_RightPunch::on_enter()
{
    original_yaw = agent->get_yaw();

    agent->setAnimAndPlay(createSampleAnimation());

    BehaviorNode::on_leaf_enter();
}

void L_RightPunch::on_update(float dt)
{
    printf("L_RightPunch\n");
    if (agent->getAnimationComp()->getCurrentFrame() == 2)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}

void L_RightPunch::on_exit()
{
    agent->stopAnim();
    // Reset agent's pitch/yaw/roll to neutral position
    agent->set_yaw(original_yaw);
}