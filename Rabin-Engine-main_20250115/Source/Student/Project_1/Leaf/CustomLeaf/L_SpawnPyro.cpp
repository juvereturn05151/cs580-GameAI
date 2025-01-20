#include "pch.h"
#include "L_SpawnPyro.h"
#include "Projects/ProjectOne.h"

L_SpawnPyro::L_SpawnPyro()
{}

void L_SpawnPyro::on_enter()
{
    // 3. Create the agent, giving it the correct AgentModel type.
    auto tree = agents->create_behavior_agent("Pyro", BehaviorTreeTypes::GoldbergBT, Agent::AgentModel::Ball);

    tree->set_position(agent->get_position());

    // 5. (optional) Set other aspects to make it start out correctly
    tree->set_color(Vec3(1.0, 1.0, 0));   // Set the tree to green

    BehaviorNode::on_leaf_enter();

    on_success();
}

void L_SpawnPyro::on_update(float dt)
{
    display_leaf_text();
}