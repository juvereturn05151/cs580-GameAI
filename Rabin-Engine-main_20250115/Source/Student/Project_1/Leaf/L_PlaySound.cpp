#include <pch.h>
#include "L_PlaySound.h"
#include "Agent/BehaviorAgent.h"

void L_PlaySound::on_enter()
{
	audioManager->PlaySoundEffect(agent->soundName);
	BehaviorNode::on_leaf_enter();
	on_success();
}