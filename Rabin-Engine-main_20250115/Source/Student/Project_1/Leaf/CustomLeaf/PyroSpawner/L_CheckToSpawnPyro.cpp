#include "pch.h"
#include "L_CheckToSpawnPyro.h"
#include "CustomScript/GlobalBlackboard.h"

void L_CheckToSpawnPyro::on_update(float dt)
{
    if (GlobalBlackboard::get_instance().get_value<bool>("CanSpawnPyro"))
    {
        on_success();
    }
    else
    {
        on_failure();
    }

    display_leaf_text();
}