#include "pch.h"
#include "L_Seek.h"

L_Seek::L_Seek()
{}

void L_Seek::on_enter()
{
    const auto& bb = agent->get_blackboard();
    targetPoint = bb.get_value<Vec3>("Click Position");
    targetPoint = Vec3(targetPoint.x, 30, targetPoint.z);

    BehaviorNode::on_leaf_enter();
}

void L_Seek::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }
    // Optional: log or display debugging info
    display_leaf_text();
}