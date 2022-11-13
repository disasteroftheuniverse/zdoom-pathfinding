
class ZNavZombieMan : ZNavAgent replaces ZombieMan
{
	Default
	{
		Health 20;
		Radius 20;
		Height 56;
		Speed 10;
		PainChance 200;
		Monster;
		+FLOORCLIP
		SeeSound "grunt/sight";
		AttackSound "grunt/attack";
		PainSound "grunt/pain";
		DeathSound "grunt/death";
		ActiveSound "grunt/active";
		Obituary "$OB_ZOMBIE";
		Tag "$FN_ZOMBIE";
		DropItem "Clip";
	}


 	States
	{
	Spawn:
		POSS AB 10 A_Look;
		Loop;
	See:
		POSS AAAABBBBCCCCDDDD 2 A_ZNavChase(null, "Missile");
		Loop;
	Missile:
		POSS E 10 A_FaceTarget;
		POSS F 8 A_PosAttack;
		POSS E 8;
		Goto See;
	Pain:
		POSS G 3;
		POSS G 3 A_Pain;
		Goto See;
	Death:
		POSS H 5;
		POSS I 5 A_Scream;
		POSS J 5 A_NoBlocking;
		POSS K 5;
		POSS L -1;
		Stop;
	XDeath:
		POSS M 5;
		POSS N 5 A_XScream;
		POSS O 5 A_NoBlocking;
		POSS PQRST 5;
		POSS U -1;
		Stop;
	Raise:
		POSS K 5;
		POSS JIH 5;
		Goto See;
	}
}