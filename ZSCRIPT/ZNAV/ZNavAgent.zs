Class ZNavAgent : Actor 
{
    /*AI manager info*/
    ZNavThinker Navigator;
    ZNavMesh NavMesh;
    ZNavNode CurNode, LastNode;

    bool UseNavigator;
    bool HasNavigator;
    int ZNavGroupID;

    /* cell space partitioning datas*/
    int targetCell;
    int LastCell, CurCell;

    bool NeedsRoute;
    ZNavRoute Route;
    int PathProgress;  
    int PathTime;
    int MaxPathTime;

    /* movement info */
    vector3 Destination;
    vector3 LastMovePos;
    double LastTurnDir;
    int movetime;

    /* leap stuff */
    bool LeapPending;
    bool isLeaping;
    QuadraticBezierCurve LeapCurve;

    /* properties */
    property UseNavigator : UseNavigator;
    
    Default
    {
        speed 16;
        radius 16;
        height 64;

        ZNavAgent.UseNavigator true;

        +SLIDESONWALLS;
        +SOLID;

        MaxDropOffHeight 32;
        MaxStepHeight 24;
    }



    override void PostBeginPlay()
    {
        super.PostBeginPlay();
        LeapCurve = QuadraticBezierCurve.Create();
        if (UseNavigator) getNavigator();
    }


    void getNavigator()
    {
        ThinkerIterator it = ThinkerIterator.Create ('ZNavThinker');
        ZNavThinker nt;

        while (nt = ZNavThinker(it.Next())) 
        {
            if ( nt )
            {
                Navigator = nt;
                break;
            }
        }

        if (!Navigator) {
            console.printf('could not get ai navigator at load time');
            return;
        }

        Navigator.subscribe( self );
        HasNavigator = true;
        CurCell = navmesh.getIndexFromPosition(pos);
        lastCell = curcell;
    }

    vector2 GetNextMoveStep(double NextMoveAngle, double speed)
    {
        return Math.SafeUnit2( RotateVector((speed, 0), NextMoveAngle) ) * speed;
    }

    vector2 GetNextMovePosition (double NextMoveAngle, double speed)
    {
        return GetNextMoveStep(NextMoveAngle, speed) + Pos.XY;
    }


    Vector3 Pathfind ( actor mo )
    {
        vector3 DestPos;

        if ( mo ) DestPos = ( mo.pos.XY, mo.floorz );
        if ( pathtime >= maxPathTime )
        {
            int CurTargetCell = NavMesh.getIndexFromPosition( DestPos );

            if ( CurTargetCell != targetCell ) 
            {
                pathtime = 0;
                maxPathTime = random( 20, 30 );
                needsRoute = true;
            } else {
                pathtime = 0;
                maxPathTime = random( 35, 55 );
            }
        }

        if ( needsRoute )
        {
            targetCell = NavMesh.getIndexFromPosition( DestPos );

            ZNavNode destnode = NavMesh.getClosestNodeInGroup ( DestPos, ZNavGroupID );
            DestPos = destnode.constrainPoint(DestPos);

            NavMesh.findPath(ZNavGroupID, pos, DestPos, route );

            if ( route.size() )
            {
                PathProgress = 1;
                needsRoute = false;
                movetime = 0;
                pathtime = 0;
                maxPathTime = random(35, 55);
            }
        }

        if ( route.size() )
        {
            pathtime++;
            DestPos = route.get(PathProgress);
        }
        
        double distSq2Dest = Math.distanceToSquared2 ( pos, DestPos );
        if ( distSq2Dest < ( (radius + 20) * (radius + 20) ) )
        {
            

            if ( !CheckMove(DestPos.XY, PCM_NOACTORS) )
            {
                needsRoute = true;
            }

            int NextStep = PathProgress;
            NextStep++;

            if ( NextStep >= route.size() )
            {
                pathtime = 0;
                bool shouldRepath = false;

                if (route.goal)
                {
                    shouldRepath = route.goal.start( self );
                }

                if ( shouldRepath )
                {
                    needsRoute = true;
                    movetime = 0;
                }

            } else {
                PathProgress = NextStep;
                movetime = 0;
            }
        }

        return DestPos;
    }

    void PathMoveTo (vector3 dest)
    {
        double LastMoveAngle = Angle;
        double NextMoveAngle = Angle;

        if ( movetime ) movetime--;

        if ( !movetime )
        {
            NextMoveAngle = Math.getAbsAngle ( pos, dest );
            double delta = DeltaAngle (LastMoveAngle, NextMoveAngle);
            if (delta != 0) LastTurnDir = delta / abs(delta);
        }

        vector2 NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
        bool moved = TryMove(NextMovePos, 0, false, null);

        if (!moved)
        {
            movetime = random (3,8);
            NextMoveAngle = Math.getAbsAngle ( pos, dest );
            NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
            moved = TryMove(NextMovePos, 0, false, null);

            if (!moved)
            {
                if (!LastTurnDir) LastTurnDir = random(0, 1) ? 1 : -1;

                for (double i = 22.5; i < 180; i+=22.5 )
                {
                    NextMoveAngle =  LastMoveAngle + ( i * LastTurnDir );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved ) 
                    {
                        LastTurnDir = 1;
                        break;
                    }

                    NextMoveAngle = LastMoveAngle - ( i * LastTurnDir );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved ) {
                        LastTurnDir = -1;
                        break;
                    }
                }
            }
        }

        Angle = NextMoveAngle;
        LastMovePos = pos;
    }

    void MoveTowards ( vector3 dest, double speed = 0.0 )
    {
        if (!speed) speed = self.speed;

        double LastMoveAngle = Angle;
        double NextMoveAngle = Angle;
        double DeltaTurnAngle;

        if ( movetime ) movetime--;

        if ( !movetime )
        {
            LastTurnDir = 0;
            movetime = random (15, 50);
            double distToDestSq = Math.distanceToSquared(pos, dest);
            double snapAngle = 22.5;

            if ( distToDestSq < (64 * 64) )
            {
                movetime = random (0, 5);
                snapAngle = 11.25;
            } 
            else if ( distToDestSq < (256 * 256) )
            {
                movetime = random (5, 15);
                snapAngle = 22.5;
            }

            DeltaTurnAngle = DeltaAngle (LastMoveAngle, Math.getAbsAngle ( pos, dest ));

            if ( abs(DeltaTurnAngle) > 44) movetime = 0;
    
            NextMoveAngle = clamp (DeltaTurnAngle, -45, 45) + LastMoveAngle;
            NextMoveAngle = SnapToAngle( NextMoveAngle, snapAngle );
        }

        vector2 NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
        bool moved = TryMove(NextMovePos, 0, false, null);

        if ( !moved )
        {
            NextMoveAngle = SnapToAngle( Math.getAbsAngle ( pos, dest ), 45 );
            NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
            moved = TryMove(NextMovePos, 0, false, null);

            if ( !moved )
            {

                if (!LastTurnDir) 
                {
                    DeltaTurnAngle = DeltaAngle (LastMoveAngle, Math.getAbsAngle ( pos, dest ));
                    LastTurnDir = ( DeltaTurnAngle > 0) ? 1 : -1;
                }

                for (double i = 22.5; i < 180; i += 22.5 )
                {
                    NextMoveAngle = SnapToAngle( LastMoveAngle + ( i * LastTurnDir ) , 22.5 );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved )  break;

                    LastTurnDir *= -1.0;
                    NextMoveAngle = SnapToAngle( LastMoveAngle + ( i * LastTurnDir ) , 22.5 );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved ) break;
                }
            }
        }

        Angle = NextMoveAngle;
        LastMovePos = pos;
    }

    void A_MoveToEx ()
    {
        if (target) {
            if (!goal) {
                goal = target;
            }
        }

        if ( !NavMesh ){
            if (goal) Destination = goal.pos;
            MoveTowards(Destination);
        } else {
            Destination = Pathfind ( goal );
            PathMoveTo( Destination );
            curCell = navmesh.getIndexFromPosition(pos);
            if (lastCell!=curCell)
            {
                navmesh.UpdateCell( lastCell, curCell, self);
            }
            lastCell = curCell;
        }

    }
    
    void A_MoveTowards()
    {
        if (goal) Destination = goal.pos;
        MoveTowards(Destination);
        bInChase = false;
    }

}


Class ZNavThinker : Thinker
{   
    ZNavMesh NavMesh;
    ZNavNode PlayerNode;

    array<ZNavAgent> agents;

    play void init( string mapname )
    {
        self.NavMesh = ZNavParser.BuildGraph( mapname, self );
    }

    play void Subscribe(ZNavAgent agent)
    {
        if ( agents.find ( agent ) == agents.size() )
        {
            agents.push(agent);
            agent.navigator = self;
            agent.route = ZNavRoute.create();
            agent.NavMesh = self.NavMesh;
            agent.ZNavGroupID = self.NavMesh.getNearestGroupID ( agent.pos, true );
            agent.needsRoute = true;
        }
    }

    play void Unsubscribe(ZNavAgent agent)
    {
        int agentIndex = agents.find(agent);
        if (  agentIndex != agents.size() )
        {
            agent.Navigator = null;
            agent.NavMesh = null;
            agents.delete(agentIndex, 1);
        }
    }
}



