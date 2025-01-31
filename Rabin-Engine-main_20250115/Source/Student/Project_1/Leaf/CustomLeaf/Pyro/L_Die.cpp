#include "pch.h"
#include "L_Die.h"

L_Die::L_Die()
{}

void L_Die::on_enter()
{
    // Trigger the jump action once
    agents->destroy_agent(agent);

    BehaviorNode::on_leaf_enter();
}

void L_Die::on_update(float dt)
{
    // Optional: log or display debugging info
    display_leaf_text();
}