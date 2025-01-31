#include "pch.h"
#include "L_MoveToHut.h"

L_MoveToHut::L_MoveToHut()
{}
//X is Forward
//Z is Side
void L_MoveToHut::on_enter()
{
    //const auto& bb = agent->get_blackboard();
    targetPoint = Vec3(100, 0, 20);//bb.get_value<Vec3>("Hut Position");

    BehaviorNode::on_leaf_enter();
}

void L_MoveToHut::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}