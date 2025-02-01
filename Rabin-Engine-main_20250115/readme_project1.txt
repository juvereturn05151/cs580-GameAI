Student Name: Ju-ve Chankasemporn

Project Name: Goldberg Entrance (Wild Version)

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
2. Affine Transformation System - To animate simple animations such as knocking on the door, left punch, and right punch
3. Global Blackboard - To store global transformation

Directions (if needed):

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
4. GoldbergBT.bht
5. PyroSpawnerBT.bht
6. PyroBT.bht
7. Bird.bht

Extra credit:
Formation

Flocking