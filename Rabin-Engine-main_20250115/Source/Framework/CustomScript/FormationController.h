#pragma once
#include "Formation.h"
#include "../Agent/BehaviorAgent.h"

class FormationController {
public:
    // Delete copy constructor and assignment operator to ensure singleton integrity
    FormationController(const FormationController&) = delete;
    FormationController& operator=(const FormationController&) = delete;

    // Get the singleton instance
    static FormationController& get_instance() 
    {
        static FormationController instance; // Thread-safe in C++11 and later
        return instance;
    }

    Formation formation;
    BehaviorAgent* leader;
    std::vector<BehaviorAgent*> agents;
    int currentSlot;
    const int slotSize = 4;
    
    void init() 
    {
        formation.createWedgeFormation(5, 10.0f);
    }

    void assignLeader(BehaviorAgent* agent)
    {
        leader = agent;
    }

    void assignFormationSlots(BehaviorAgent* newAgent) 
    {
        agents.push_back(newAgent);
    }

    void ChangeFormationToLine(BehaviorAgent* agent)
    {
        leader = agent;
        formation.createLineFormation(5, 2.0f);
    }

    void update() {
        for (size_t i = 0; i < agents.size(); ++i) {
            if (agents[i] != nullptr && leader != nullptr) 
            {
                Vec3 targetPos;
                if (formation.formationType == FormationType::Wedge) 
                {
                    targetPos = leader->get_position() + Vec3(3.0f, 0, 3.0f) + formation.slots[i].offset;
                }
                else 
                {
                    targetPos = leader->get_position()  + formation.slots[i].offset;
                }

                agents[i]->get_blackboard().set_value("target", targetPos);
            }
        }
    }

private:
    // Private constructor to prevent external instantiation
    FormationController() = default;
};