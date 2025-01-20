#include "pch.h"
#include "L_CheckIfGoldbergIsFree.h"

#include "CustomScript/GlobalBlackboard.h"

void L_CheckIfGoldbergIsFree::on_update(float dt)
{
    if (GlobalBlackboard::get_instance().get_value<bool>("GoldbergisFree"))
    {
        on_success();
    }
    else
    {
        on_failure();
    }

    display_leaf_text();
}