#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"
#include "CustomScript/GlobalBlackboard.h"
#include "../CustomScript/FormationController.h"
void ProjectOne::setup()
{
    auto& blackboard = GlobalBlackboard::get_instance();
    blackboard.set_value("GoldbergIsSpawn", false);
    blackboard.set_value("GoldbergIsStandby", false);
    blackboard.set_value("CanSpawnPyro", false);
    blackboard.set_value("GoldbergisFree", false);

    // Create an agent (using the default "Agent::AgentModel::Man" model)
    auto man = agents->create_behavior_agent("ExampleAgent", BehaviorTreeTypes::GuardBT);
    man->set_position(Vec3(100, 0, 50));
    man->set_yaw(-270.0);
    man->get_blackboard().set_value("originPos", man->get_position());
    man->soundName = L"Assets\\Audio\\knock.wav";
    FormationController::get_instance().assignLeader(man);

    //Spawn Minions GuardBT
    for (int i = 0; i < 5; i++) 
    {
        auto man = agents->create_behavior_agent("ExampleAgent", BehaviorTreeTypes::GuardFollower);
        man->set_position(Vec3(100, 0, 50));
        man->set_yaw(-270.0);
        man->get_blackboard().set_value("originPos", man->get_position());
        man->soundName = L"Assets\\Audio\\knock.wav";
        FormationController::get_instance().assignFormationSlots(man);
    }




    // You can change properties here or at runtime from a behavior tree leaf node
    // Look in Agent.h for all of the setters, like these:
    // man->set_color(Vec3(1, 0, 1));
    // man->set_scaling(Vec3(7,7,7));
    // man->set_position(Vec3(100, 0, 100));

    // Create an agent with a different 3D model:
    // 1. (optional) Add a new 3D model to the framework other than the ones provided:
    //    A. Find a ".sdkmesh" model or use https://github.com/walbourn/contentexporter
    //       to convert fbx files (many end up corrupted in this process, so good luck!)
    //    B. Add a new AgentModel enum for your model in Agent.h (like the existing Man or Tree).
    // 2. Register the new model with the engine, so it associates the file path with the enum
    //    A. Here we are registering all of the extra models that already come in the package.
    Agent::add_model("Assets\\tree.sdkmesh", Agent::AgentModel::Tree);
    Agent::add_model("Assets\\car.sdkmesh", Agent::AgentModel::Car);
    Agent::add_model("Assets\\bird.sdkmesh", Agent::AgentModel::Bird);
    Agent::add_model("Assets\\ball.sdkmesh", Agent::AgentModel::Ball);
    Agent::add_model("Assets\\hut.sdkmesh", Agent::AgentModel::Hut);
    // 3. Create the agent, giving it the correct AgentModel type.
    //auto tree = agents->create_behavior_agent("ExampleAgent2", BehaviorTreeTypes::Example, Agent::AgentModel::Tree);
    // 4. (optional) You can also set the pitch of the model, if you want it to be rotated differently
    //tree->set_pitch(PI / 2);
    // 5. (optional) Set other aspects to make it start out correctly
    //tree->set_color(Vec3(0, 0.5, 0));   // Set the tree to green

    auto hut = agents->create_behavior_agent("hut", BehaviorTreeTypes::Idle, Agent::AgentModel::Hut);
    hut->set_position(Vec3(120, 0, -50));

    auto pyroSpawner = agents->create_behavior_agent("PyroSpawner", BehaviorTreeTypes::PyroSpawnerBT, Agent::AgentModel::Car);
    pyroSpawner->set_scaling(Vec3(0.5, 0.5, 0.5));
    pyroSpawner->set_position(Vec3(50, 0, 25));
    pyroSpawner->soundName = L"Assets\\Audio\\clicker.wav";
    auto pyroSpawner2 = agents->create_behavior_agent("PyroSpawner", BehaviorTreeTypes::PyroSpawnerBT, Agent::AgentModel::Car);
    pyroSpawner2->set_scaling(Vec3(0.5, 0.5, 0.5));
    pyroSpawner2->set_position(Vec3(50, 0, 75));
    pyroSpawner2->soundName = L"Assets\\Audio\\clicker.wav";

    // You can technically load any map you want, even create your own map file,
    // but behavior agents won't actually avoid walls or anything special, unless you code
    // that yourself (that's the realm of project 2)
    terrain->goto_map(3);

    // You can also enable the pathing layer and set grid square colors as you see fit.
    // Works best with map 0, the completely blank map
    terrain->pathLayer.set_enabled(true);
    terrain->pathLayer.set_value(0, 0, Colors::Red);

    // Camera position can be modified from this default
    auto camera = agents->get_camera_agent();
    camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
    camera->set_pitch(0.610865); // 35 degrees

    // Sound control (these sound functions can be kicked off in a behavior tree node - see the example in L_PlaySound.cpp)
    audioManager->SetVolume(0.5f);
   // audioManager->PlaySoundEffect(L"Assets\\Audio\\retro.wav");
    audioManager->PlayMusic(L"Assets\\Audio\\Goldberg Entrance Video.wav");
    // Uncomment for example on playing music in the engine (must be .wav)
    // audioManager->PlayMusic(L"Assets\\Audio\\motivate.wav");
    // audioManager->PauseMusic(...);
    // audioManager->ResumeMusic(...);
    // audioManager->StopMusic(...);
}