#include "pch.h"
#include "L_FreeGoldberg.h"

#include "CustomScript/GlobalBlackboard.h"

L_FreeGoldberg::L_FreeGoldberg() {}

void L_FreeGoldberg::on_enter()
{
    timer = 1.0f;
    GlobalBlackboard::get_instance().set_value("GoldbergisFree", true);
    BehaviorNode::on_leaf_enter();
}

void L_FreeGoldberg::on_update(float dt)
{
    timer -= dt;

    if (timer < 0.0f)
    {
        GlobalBlackboard::get_instance().set_value("GoldbergisFree", true);
        on_success();
    }

    display_leaf_text();
}