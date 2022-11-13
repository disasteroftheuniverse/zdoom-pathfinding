Class ZNavDebugVis
{
    static const name PathColors[] = 
    {
        "ZNavRed",
        "ZNavOrange",
        "ZNavYellow",
        "ZNavGold",
        "ZNavGreen",
        "ZNavBlue",
        "ZNavPurple"
    };

    name PathColor;
    array<ZNavLineVis>Lines;
    array<ZNavSpotIcon>Icons;
    bool hasPathColor;

    static ZNavDebugVis Create ()
    {
        return ZNavDebugVis(new ("ZNavDebugVis"));
    }

    play void setPathColor()
    {
        if (hasPathColor) return;
        PathColor = PathColors[random(0, 6)];
        hasPathColor = true;
    }

    play void AddLine(actor start, actor end)
    {
        setPathColor();
        if (!start || !end) return;

        vector3 MidPos = Math.Lerp(start.pos, end.pos, 0.5);
        let visLine = ZNavLineVis(Actor.Spawn("ZNavLineVis", MidPos));
        if (visLine)
        {
            Lines.push(visLine);
            visLine.angle = visLine.angleTo(start) + 90.;
            float ptch = Math.getPitch(visLine.pos, start.pos);
            visLine.A_SetRoll ( -ptch, 0, AAPTR_DEFAULT );
            visLine.Scale.X = visLine.Distance3D(start);
            visLine.A_SetTranslation(PathColor);
        }
    }

    play void AddLinePos(vector3 start, vector3 end)
    {
        setPathColor();

        //if (!start || !end) return;
        vector3 diff = end - start;
        vector3 MidPos = Math.Lerp(start, end, 0.5);

        let visLine = ZNavLineVis(Actor.Spawn("ZNavLineVis", MidPos));

        if (visLine)
        {
            Lines.push(visLine);
            visLine.angle = Math.getAbsAngle(start, end) + 90;
            //visLine.angleTo(start) + 90.;
            float ptch = Math.getPitch(visLine.pos, start);
            visLine.A_SetRoll ( -ptch, 0, AAPTR_DEFAULT );
            visLine.Scale.X = diff.length();
            visLine.A_SetTranslation(PathColor);
        }
    }

    play void AddSpot(vector3 position, double scale = 1.0)
    {
        setPathColor();
        ZNavSpotIcon pathIcon = ZNavSpotIcon(Actor.Spawn("ZNavSpotIcon", position));
        if (pathIcon)
        {
            Icons.push(pathIcon);
            pathIcon.A_SetTranslation(PathColor);
            pathIcon.A_SetScale(scale, scale);
        }
    }

    play void AddLinesToSpots()
    {
        int MaxIcons = Icons.size();
        for (int i = 0; i < MaxIcons; i++)
        {
            int j = i + 1;
            if ( j >= MaxIcons) break;
            ZNavSpotIcon last = Icons[i];
            ZNavSpotIcon next = Icons[j];
            AddLine (last, next);
        }
    }

    play void ClearLines()
    {
        for (int i = 0; i < Lines.Size(); i++)
        {
            ZNavLineVis pvline = Lines[i];
            if (pvline) pvline.destroy();
        }

        for (int i = 0; i < Icons.Size(); i++)
        {
            ZNavSpotIcon pathIcon = Icons[i];
            if (pathIcon) pathIcon.destroy();
        }

        Lines.clear();
        Icons.clear();
    }
}

Class ZNavLineVis : Actor
{
    Default
    {
        +NOCLIP;
        +NOBLOCKMAP;
        +NOGRAVITY;
        +WALLSPRITE;
        +ROLLSPRITE;
        +BRIGHT;
    }

    States 
    {
        Spawn:
            ZNAV A 1 Bright;
            ZNAV AAA 800 Bright;
            Stop;
    }
}

Class ZNavSpotIcon : Actor
{
    Default
    {
        +NOCLIP;
        +NOBLOCKMAP;
        +NOGRAVITY;
        +BRIGHT;
        +FORCEXYBILLBOARD;
        Scale 0.5;
    }

    States
    {
        Spawn:
            ZNAV B 1 Bright;
            ZNAV BBB 800 Bright;
            Stop;
    }
}