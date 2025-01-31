#include "pch.h"
#include "L_MoveToOrigin.h"

#include "Agent/BehaviorAgent.h"

void L_MoveToOrigin::on_enter()
{

    // set animation, speed, etc

    // grab the target position from the blackboard
    const auto& bb = agent->get_blackboard();
    targetPoint = bb.get_value<Vec3>("originPos");

    //agent->look_at_point(targetPoint);
    //printf("moveToOrigin %f %f %f\n", targetPoint.x, targetPoint.y, targetPoint.z);
    BehaviorNode::on_leaf_enter();
}

void L_MoveToOrigin::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}