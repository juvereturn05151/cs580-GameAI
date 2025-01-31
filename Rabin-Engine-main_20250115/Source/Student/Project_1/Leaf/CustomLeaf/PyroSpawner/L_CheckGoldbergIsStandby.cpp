#include "pch.h"
#include "L_CheckGoldbergIsStandby.h"

#include "CustomScript/GlobalBlackboard.h"

void L_CheckGoldbergIsStandby::on_update(float dt)
{
    if (GlobalBlackboard::get_instance().get_value<bool>("GoldbergIsStandby"))
    {
        on_success();
    }
    else
    {
        on_failure();
    }

    display_leaf_text();
}