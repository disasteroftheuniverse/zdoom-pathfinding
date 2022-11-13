class NavHelper : Actor abstract
{
    znavnode node;
    Default
    {
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7
        Scale 0.33;
        +NOBLOCKMAP;
        +NOGRAVITY;
    }
    
    void addNode( znavnode node)
    {
        self.node = node;
    }

    bool start ( znavagent agent )
    {
        return true;
    }

    virtual void init()
    {
        //
    }

    override void PostBeginPlay()
    {
        super.PostBeginPlay();
        init();
    }
}

class NavLink : NavHelper abstract
{
    NavLink parent;
    NavLink link;

    static NavLink getLinkByTID ( int tag )
    {
        ThinkerIterator it = ThinkerIterator.Create ('NavLink');
        NavLink nt;
        while (nt = NavLink(it.Next())) 
        {
            if ( nt && nt.tid == tag) return nt;
        }
        return null;
    }
}

class NavLeap : NavLink
{
    znavagent agent;
    int agentWaitTime;

    navlink arcnode;
    navlink landingnode;

    Default
    {
        Tag "AI Leap Spot";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Leap Spot"
        //$Sprite PNODM0

        //$Arg0 Next destination
        //$Arg0Type 14
        //$Arg0 "Link 1"
        //$Arg0Default -1
    }

    override void init()
    {
        if ( args[0] > 0 )
        {
            arcnode = NavLink.getLinkByTID ( args[0] );

            if ( arcnode )
            {
                if ( arcnode.args[0] > 0 )
                {
                    landingnode = NavLink.getLinkByTID ( arcnode.args[0] );

                    if (landingnode)
                    {
                        self.link = arcnode;
                        arcnode.link = landingnode;
                        arcnode.parent = self;
                        landingnode.parent = arcnode;
                    }
                }
            }

        } else {
            console.printf('\c[red]nav node is missing links!\cl');
        }
    }

    // repath, interrupt
    bool start (znavagent agent)
    {
        bool agentLeaping = addAgent(agent);
        if (agentLeaping)
        {
            vector3 startPos, endPos, midPos;
            if (agent) 
            {
                startPos = agent.pos;
                endPos = landingnode.pos;
                midPos = Math.Lerp( startPos, endPos, 0.5 );
                midPos.z = arcnode.z;
                agent.LeapCurve.init ( startPos, midPos, endPos );
            }
            return false;
        }
        return false;
    }

    bool addAgent(znavagent agent)
    {
        if (agentWaitTime) return false;

        self.agent = agent;
        agentWaitTime = 21;
        return true;
    }

    override void tick()
    {
        super.tick();
        if ( agentWaitTime ) agentWaitTime--;
    }
}

class NavArc : NavLink
{
    Default
    {
        Tag "AI Leap Arc";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Leap Arc"
        //$Sprite PNODH0

        //$Arg0 Next destination
        //$Arg0Type 14
        //$Arg0 "Link 1"
        //$Arg0Default -1
    }
}

class NavLanding : NavLink
{
    Default
    {
        Tag "AI Landing Spot";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Landing Spot"
        //$Sprite PNODR0
    }
}

class NavObstacle : NavHelper
{
    Default
    {
        Tag "AI Obstacle";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Obstacle"
        //$Sprite PNODN0
    }
}

class NavAvoid : NavHelper
{
    Default
    {
        Tag "AI Avoid";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Avoid"
        //$Sprite PNODO0
    }
}

class NavAction: NavHelper
{
    Default
    {
        Tag "AI Action";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Action"
        //$Sprite PNODQ0
    }
}

class NavPlatform: NavHelper
{
    Default
    {
        Tag "AI Platform Area";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Platform Area"
        //$Sprite PNODS0

        //$Arg0 "Size"
        //$Arg0Type 23
        //$Arg0Default 64

    }
}

class NavTarget: NavHelper
{
    Default
    {
        Tag "AI Target";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Target"
        //$Sprite PNODP0
    }
}

class NavTeleport : NavHelper
{
    Default
    {
        Tag "AI Teleport Spot";
        RenderStyle "none";
        //$Category "AI Navigation"
        //$IgnoreRenderstyle
        //$color 7

        //$Title "AI Teleport Spot"
        //$Sprite PNODR0
    }
}