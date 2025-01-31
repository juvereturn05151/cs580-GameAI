#include "pch.h"
#include "L_CheckGoldbergIsSpawn.h"
#include "CustomScript/GlobalBlackboard.h"

void L_CheckGoldbergIsSpawn::on_update(float dt)
{
    if (GlobalBlackboard::get_instance().get_value<bool>("GoldbergIsSpawn"))
    {
        on_success();
    }
    else
    {
        on_failure();
    }

    display_leaf_text();
}