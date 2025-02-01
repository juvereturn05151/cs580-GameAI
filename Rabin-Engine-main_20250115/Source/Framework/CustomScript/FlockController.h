#pragma once
#include "../Agent/BehaviorAgent.h"

class FlockController {
public:
    // Delete copy constructor and assignment operator to ensure singleton integrity
    FlockController(const FlockController&) = delete;
    FlockController& operator=(const FlockController&) = delete;

    // Get the singleton instance
    static FlockController& get_instance() {
        static FlockController instance; // Thread-safe in C++11 and later
        return instance;
    }

    std::vector<BehaviorAgent*> boids;
    float maxSpeed = 2.0f;
    float perceptionRadius = 2.0f;
    float seekWeight = 2.0f;

    void init() {

    }

    void addBoid(BehaviorAgent* newAgent) {
        boids.push_back(newAgent);
    }

private:
    // Private constructor to prevent external instantiation
    FlockController() = default;
};