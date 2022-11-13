Class ZNavAgent : Actor 
{
    bool user_debugpath;
    actor LastKnownTarget;

    /*AI manager info*/
    ZNavThinker Navigator;
    ZNavMesh NavMesh;
    ZNavNode CurNode, LastNode;

    bool UseNavigator;
    bool HasNavigator;
    int ZNavGroupID;

    //debug - view routes
    ZNavDebugVis RouteVis;

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

        ZNavAgent.UseNavigator true;

        +SLIDESONWALLS;
        +SOLID;
        +NOINFIGHTING;

        MaxDropOffHeight 32;
        MaxStepHeight 24;
    }

    //  this is just here to make enemies attack less so you can 
    //  see if they follow their paths or not
    //  dont use this in production
    bool ZNavCheckMissileRange()
    {
        if ( NerfChanceCount ) {

            NerfChanceCount--;
            return false;
        }
        else
        {
            NerfChanceCount = 32;
            return CheckMissileRange();
        }
    }
    int NerfChanceCount;

    override void PostBeginPlay()
    {
        super.PostBeginPlay();
        LeapCurve = QuadraticBezierCurve.Create();
        if (UseNavigator) getNavigator();
    }

    //check to see if this map has a navmesh, and if so, get a pointer to it
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

        if (!Navigator) 
        {
            return;
        }

        Navigator.subscribe( self );
        RouteVis = ZNavDebugVis.Create();
        HasNavigator = true;
        CurCell = navmesh.getIndexFromPosition(pos);
        lastCell = curcell;
    }

    //given an angle and speed, get the coordinates of my next move relative to my current position
    vector2 GetNextMoveStep(double NextMoveAngle, double speed)
    {
        return Math.SafeUnit2( RotateVector((speed, 0), NextMoveAngle) ) * speed;
    }

    //given an angle and speed, get the absolute coordinates of my next move
    vector2 GetNextMovePosition (double NextMoveAngle, double speed)
    {
        return GetNextMoveStep(NextMoveAngle, speed) + Pos.XY;
    }

    //visualize my route
    void VisualizeRoute()
    {
        if (!RouteVis) return;

        RouteVis.ClearLines();

        for (int i = 0; i < route.size(); i++ )
        {
            vector3 v = route.get(i);
            RouteVis.AddSpot(v, 1.0);
        }

        RouteVis.AddLinesToSpots();
    }

    //returns a vector that I can move towards
    Vector3 Pathfind ( actor mo )
    {
        vector3 DestPos;

        //set the destination point to target actor's position
        if ( mo ) DestPos = ( mo.pos.XY, mo.floorz );

        //if we have been traveling for a while, check to see if we need a new path
        if ( pathtime >= maxPathTime )
        {
            //check to see if the target has moved by looking up the cell that the target is in
            //cells are basically just like the blockmap
            int CurTargetCell = NavMesh.getIndexFromPosition( DestPos );

            //if the target's cell is not the same as the last time checked, request a new route
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

        //if I need a new route, get a new route
        if ( needsRoute )
        {
            targetCell = NavMesh.getIndexFromPosition( DestPos ); //remember the target's current cell so I can check it later

            ZNavNode destnode = NavMesh.getClosestNodeInGroup ( DestPos, ZNavGroupID );  //figure out what node I will end in

            DestPos = destnode.constrainPoint(DestPos); //make sure destination is on the mesh by snapping it to the node

            NavMesh.findPath(ZNavGroupID, pos, DestPos, route ); //find me a route, a list of points to move thru

            if ( route.size() ) //if I got a route..
            {
                if (user_debugpath) VisualizeRoute();

                PathProgress = 1; //set my destination to the first point on the path
                needsRoute = false; //dont request a new route yet
                movetime = 0;   //how long i should move this direction before I can turn (0 means always turn to face movement direction)
                pathtime = 0;   //start counting how long I am on a route
                maxPathTime = random(35, 55);   //how long i should follow this route before I check for a new one
            }
        }

        if ( route.size() ) //if I have a route...
        {
            pathtime++; //count how long I've been on this route
            DestPos = route.get(PathProgress);  //get the next position on my route
        }
        

        if ( NavMesh.CheckArrival(self, DestPos) )  //if I have arrived...
        {
            
            if ( !CheckMove(DestPos.XY, PCM_NOACTORS) ) //if I can't actually get to my next position, request a new route
            {
                needsRoute = true;
            }

            // check to see if I can advance my progess or not
            int NextStep = PathProgress;    
            NextStep++;

            // if the next step would be larger than the route size, i have reached the end of my route
            if ( NextStep >= route.size() )
            {
                pathtime = 0;
                bool shouldRepath = true;   //you can replace this with some kind of function if you want to

                if ( shouldRepath )
                {
                    needsRoute = true;
                    movetime = 0;
                }

            } else {
                PathProgress = NextStep;    //if there are still more points on the route, increase the progress so I can get the next position
                movetime = 0;
            }
        }

        //return the point where I must move to next
        return DestPos;
    }

    //how to move if I'm on a route
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

    //how to move if I'm not on a route 
    //(this is very similar to classic A_Chase Behavior)
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
            NextMoveAngle = Math.SnapToAngle( NextMoveAngle, snapAngle );
        }

        vector2 NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
        bool moved = TryMove(NextMovePos, 0, false, null);

        if ( !moved )
        {
            NextMoveAngle = Math.SnapToAngle( Math.getAbsAngle ( pos, dest ), 45 );
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
                    NextMoveAngle = Math.SnapToAngle( LastMoveAngle + ( i * LastTurnDir ) , 22.5 );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved )  break;

                    LastTurnDir *= -1.0;
                    NextMoveAngle = Math.SnapToAngle( LastMoveAngle + ( i * LastTurnDir ) , 22.5 );
                    NextMovePos = GetNextMovePosition( NextMoveAngle, speed );
                    moved = TryMove(NextMovePos, 0, false, null);
                    if ( moved ) break;
                }
            }
        }

        Angle = NextMoveAngle;
        LastMovePos = pos;
    }

    //decide how to move and then move there
    void A_MoveToEx ()
    {
        //if my target changed, get a new route
        if (target != null && target != LastKnownTarget)
        {
            NeedsRoute = true;
        }

        //if I don't have a goal, make my goal my target
        if (target) {
            if (!goal) {
                goal = target;
            }
        }

        
        if ( !NavMesh )
        {  // if I don't have a nav mesh, move similarly to A_Chase

            if (goal) Destination = goal.pos;
            MoveTowards(Destination);

        } else {    // if I do have a nav mesh...
            
            Destination = Pathfind ( goal ); // ...find out what point I should move to next...

            PathMoveTo( Destination );  // ... and try to move towards that point


            //Update my current cell, so that it's easier to 
            //find me on the map if someone searches for me
            curCell = navmesh.getIndexFromPosition(pos);  
            
            //if I changed cells, inform the navmesh of the change
            if (lastCell!=curCell)
            {
                navmesh.UpdateCell( lastCell, curCell, self);
            }
            lastCell = curCell;
        }

    }

    //just move towards another actor
    void A_MoveTowards()
    {
        if (goal) Destination = goal.pos;
        MoveTowards(Destination);
        bInChase = false;
    }

    //a clunky chase replacement for A_Chase. 
    //create something better for your project
    virtual bool A_ZNavChase( statelabel meleestate = null, statelabel missilestate = null)
    {
        if (bInConversation) return false;
        if (bInChase) return false;
        bInChase = true;

        if (!target)
        {
            LookForPlayers (true);

            if (!target)
            {
                bInChase = false;
                setIdle ();
                return false;
            }
        }

        if (reactiontime)
        {
            reactiontime--;
        }

        if ( meleestate )
        {
            if ( CheckMeleeRange () )
            {
                bInChase = false;
                bInCombat = true;
                setStateLabel(meleestate);
                return false;
            }
        }

        if ( missilestate && !bJustAttacked )
        {
            if ( ZNavCheckMissileRange() )
            {
                bJustAttacked = true;
                bInChase = false;
                bInCombat = true;
                setStateLabel(missilestate);
                return false;
            }
        }

        if (bJustAttacked) bJustAttacked = false;

        A_MoveToEx();

		if (random(0, 255) < 2)
		{
			PlayActiveSound ();
		}

        LastKnownTarget = target;
        bInChase = false;

        return true;
    }

    //on death, clean up objects
    override void Die(Actor source, Actor inflictor, int dmgflags, Name MeansOfDeath)
    {
        if (RouteVis) 
        { 
            RouteVis.ClearLines();
            RouteVis.destroy();
        }

        if ( route ) route.dispose();
        if ( LeapCurve ) LeapCurve.destroy();
        if (Navigator) Navigator.Unsubscribe(self);

        super.Die(source, inflictor, dmgflags, MeansOfDeath);
    }

}

/* 
    if your map has a navmesh 
    a navthinker will get created automatically

    the navthinker just gets the navmesh data and gives it to agents
    there should only ever be 1 navthinker in a level at a time

    it also keeps track of all agents

    do whatever you want with that information
    there really isn't much reason to 
    mess with this at all
*/

Class ZNavThinker : Thinker
{   
    ZNavMesh NavMesh;
    ZNavNode PlayerNode;

    array<ZNavAgent> agents;

    play void init( string mapname )
    {
        self.NavMesh = ZNavParser.BuildGraph( mapname, self );
    }


    //register an agent and keep track of it
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

    ///deregister an agent and no longer track it
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



