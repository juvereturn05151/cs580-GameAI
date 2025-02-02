Student Name: Ju-ve Chankasemporn

Project Name: Goldberg's Entrance 

What I implemented:

I implemented a wrestler named Goldberg's entrance.

Referenced in this video: https://www.youtube.com/watch?v=pEOQrsph3po&t=81s

1st Step: Design.
- I chose Goldberg's Entrance as my project because it has cool sequences on his entrance. Eg. 1. The sequence when the guard knocks the door, and he comes out. 2. When he punches around to spawn Pyro(or firework). 
- With his entrance, I see that I can implement a lot of behavior tree nodes as well as a formation system for extra credits at the same time, this is why I choose to do this.

2nd Step: Implementing needed features.
- Since the engine doesn't have many features. I have to develop and import many features by myself. 
Including 
1. Physics Component System - To make the character jump and make the pyro(or firework) go up in the sky.
2. Affine Transformation System - To animate simple animations such as knocking on the door, left punch, and right punch.
3. Global Blackboard - To store global transformation.

3rd Step: Implementing Leaf Nodes and Behavior Trees
1. I start setting up by creating a guard's behavior tree that only moves to the Hut to trigger(or spawn Goldberg).
2. Then I build Goldberg's behavior tree which does his cool stuff such as walking and punching.
3. Then I built Pyro Spawner and Pyro(or fireworks)'s behavior trees.
4. To make all the agents work together, I utilize Global Blackboard to store important information such as when Goldberg is free and when Goldberg is on standby as Booleans. With this information, I create leaf nodes that wait until the certain Boolean is true, then do their actions.

4th Step: Implementing extra credits
1. After I was done with my own version of Goldberg's entrance sequences, I started doing extra credits.
2. For formation, I created Guard Followers' behavior tree that only follows the normal guard and then switch to follow Goldberg at the end.
3. For flocking, I just implemented birds that are independent from Goldberg's entrance.

Directions (if needed):
1. At the start of the project, the guard and his followers will go to the Hut, and knock on the door to call Goldberg.
2. Once the Goldberg is spawned, he will go to the position between the pyro spawners.
3. There, he will punch left and right, and the pyro spawners will spawn pyro(or fireworks).
4. After he jumps, he is controllable by using the mouse click to go to target position.
5. After he is controllable, the guard followers will follow him.

Note - bird agents are independent agents that are controllable at the start.


What I liked about the project and framework:
- It is very easy to use.
- It is very customizable on the lower level.

What I disliked about the project and framework:
- There are not a lot of resources

Any difficulties I experienced while doing the project:

Hours spent: 28 hours (approximately)
I didn't sleep on the weekend that this homework was given.

New selector node (name): - (no new ones)

New decorator nodes (names):  - (no new ones)

10 total nodes (names):

For Goldberg:
1. L_Jump.h
2. L_MoveOutOfHut
3. L_CenterItself
4. L_LeftPunch
5. L_RightPunch
6. L_FreeGoldberg

For Guard
7. L_MoveToHut
8. L_KnockTheDoor
9. L_SpawnAgent
10. L_CheckGoldbergIsSpawn
12. L_MoveToOrigin

For Pyro Spawner
13. L_SpawnPyro
14. L_AutoRotate
15. L_CheckGoldbergIsStandby
16. L_CheckToSpawnPyro

For Pyro
17. L_Die
18. L_FireUpward

Shared nodes(used by more than 2 Behavior Trees)
19. L_MoveToTarget
20. L_CheckIfGoldbergIsFree

For Bird
21. L_Seek

4 Behavior trees (names):

1. GuardBT.bht
2. GuardFollower.bht
3. Idle.bht 
4. GoldbergBT.bht (this one has 3 nodes deep)
5. PyroSpawnerBT.bht
6. PyroBT.bht
7. Bird.bht

Extra credit:
Bug Report - 
Read the information and e-mail evidence here.
https://docs.google.com/document/d/1gPjw0_6xb9njpCHw4gTa3XIzgZyQnaXR1_pfjHjzrJ4/edit?tab=t.0

Formation:
Important Scripts - 1. Formation.h 2. FormationController.h
Agent's BT - GuardFollower.bht
Implementation Details: 
1. I scripted Formation store the formation for Line and Wedge Formation.
2. FormationController stores the agent that is related to the formation including the leader and followers
3. FormationController sets the target position related to the leader as either Wedge or Line formation.
4. Then, GuardFollower.bht that moves toward that position set by Formation Controller
Note - the guard followers will go with Wedge formation at the start, then after Goldberg is free, they will go as Line formation.

Flocking
Important Scripts - 1. FlockController.h 2. PhysicsComponent.h
Agent's BT - Bird.bht
Leaf Node - L_Seek
Implementation Details: 
1. FlockController stores behavior agents and perception radius.
2. Physics component is where the steering behaviors are implemented(Sepation, Alignment, Cohesion, and Seek).
3. I call the implementations in BehaviorAgent's Seek function.
4. The Bird then uses L_Seek to go to where the click position is.
