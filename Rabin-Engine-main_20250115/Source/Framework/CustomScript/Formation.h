#pragma once
#include <iostream>
#include "../Misc/NiceTypes.h" 
#include <vector>

enum FormationType 
{
    Line, Wedge
};

struct FormationSlot {
    Vec3 offset;
    bool occupied = false;
};

class Formation
{
public:
    std::vector<FormationSlot> slots;
    FormationType formationType;

    void createLineFormation(int numAgents, float spacing) {
        formationType = FormationType::Line;
        slots.clear();
        for (int i = 0; i < numAgents; ++i) 
        {
            slots.push_back({ Vec3(i * spacing, 0, 0), false });
        }
    }

    void createWedgeFormation(int numAgents, float spacing) {
        formationType = FormationType::Wedge;
        slots.clear();
        int mid = numAgents / 2;
        for (int i = 0; i < numAgents; ++i) {
            float x = (i - mid) * spacing;
            float z = abs(i - mid) * spacing * 0.5f;
            slots.push_back({ Vec3(x, 0, z), false });
        }
    }
};

