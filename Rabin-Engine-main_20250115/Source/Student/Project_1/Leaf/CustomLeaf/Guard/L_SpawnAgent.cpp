#include "pch.h"
#include "L_SpawnAgent.h"
#include "Projects/ProjectOne.h"
#include "CustomScript/GlobalBlackboard.h"
#include "Agent/BehaviorAgent.h"
L_SpawnAgent::L_SpawnAgent()
{}

void L_SpawnAgent::on_enter()
{
    // 3. Create the agent, giving it the correct AgentModel type.
    auto goldberg = agents->create_behavior_agent("ExampleAgent2", BehaviorTreeTypes::GoldbergBT, Agent::AgentModel::Man);
    
    goldberg->set_position(Vec3(80,0, 20));
    goldberg->get_blackboard().set_value("target", Vec3(0, 0, 50));
    // 5. (optional) Set other aspects to make it start out correctly
    goldberg->set_color(Vec3(1.0, 1.0, 0));   // Set the tree to green
    goldberg->soundName = L"Assets\\Audio\\malesound.wav";;
    GlobalBlackboard::get_instance().set_value("CanSpawnPyro", false);
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