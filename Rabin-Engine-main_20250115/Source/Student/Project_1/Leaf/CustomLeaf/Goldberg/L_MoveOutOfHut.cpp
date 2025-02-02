#include "pch.h"
#include "L_MoveOutOfHut.h"

L_MoveOutOfHut::L_MoveOutOfHut()
{}
//X is Forward
//Z is Side
void L_MoveOutOfHut::on_enter()
{
    //const auto& bb = agent->get_blackboard();
    targetPoint = Vec3(65, 0, 47.5);//bb.get_value<Vec3>("Hut Position");

    BehaviorNode::on_leaf_enter();
}

void L_MoveOutOfHut::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    // Optional: log or display debugging info
    display_leaf_text();
}