#include "pch.h"
#include "L_SpawnAgent.h"
#include "Projects/ProjectOne.h"

L_SpawnAgent::L_SpawnAgent()
{}

void L_SpawnAgent::on_enter()
{
    // 3. Create the agent, giving it the correct AgentModel type.
    auto tree = agents->create_behavior_agent("ExampleAgent2", BehaviorTreeTypes::Example, Agent::AgentModel::Tree);
    
    tree->set_position(Vec3(50,0, 50));
    
    //4. (optional) You can also set the pitch of the model, if you want it to be rotated differently
    tree->set_pitch(PI / 2);
    // 5. (optional) Set other aspects to make it start out correctly
    tree->set_color(Vec3(0, 0.5, 0));   // Set the tree to green

    BehaviorNode::on_leaf_enter();

    on_success();
}

void L_SpawnAgent::on_update(float dt)
{
    display_leaf_text();
}