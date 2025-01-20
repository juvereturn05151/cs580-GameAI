#include "pch.h"
#include "L_SpawnAgent.h"
#include "Projects/ProjectOne.h"
#include "CustomScript/GlobalBlackboard.h"
L_SpawnAgent::L_SpawnAgent()
{}

void L_SpawnAgent::on_enter()
{
    // 3. Create the agent, giving it the correct AgentModel type.
    auto tree = agents->create_behavior_agent("ExampleAgent2", BehaviorTreeTypes::GoldbergBT, Agent::AgentModel::Man);
    
    tree->set_position(Vec3(100,0, 20));
    
    // 5. (optional) Set other aspects to make it start out correctly
    tree->set_color(Vec3(1.0, 1.0, 0));   // Set the tree to green

    BehaviorNode::on_leaf_enter();

    on_success();
}

void L_SpawnAgent::on_update(float dt)
{
    display_leaf_text();
}

void L_SpawnAgent::on_exit() 
{
    GlobalBlackboard::get_instance().set_value("GoldbergIsSpawn", true);
}