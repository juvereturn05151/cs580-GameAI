#include "pch.h"
#include "L_Seek.h"

L_Seek::L_Seek()
{}

void L_Seek::on_enter()
{
    BehaviorNode::on_leaf_enter();
}

void L_Seek::on_update(float dt)
{
    // Optional: log or display debugging info
    display_leaf_text();
}