/// <summary>
/// Manages debug visualization for navigation paths, including lines between points
/// and spot icons at path nodes. Each instance is assigned a random path color that
/// is shared by all visuals it spawns.
/// </summary>
Class ZNavDebugVis
{
    /// <summary>
    /// Available translation names used to color path visuals.
    /// One is randomly selected per ZNavDebugVis instance.
    /// </summary>
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

    /// <summary>Translation name applied to all spawned visuals.</summary>
    name PathColor;

    /// <summary>All spawned line visualization actors.</summary>
    array<ZNavLineVis> Lines;

    /// <summary>All spawned spot/icon visualization actors.</summary>
    array<ZNavSpotIcon> Icons;

    /// <summary>Whether a path color has already been assigned.</summary>
    bool hasPathColor;

    /// <summary>
    /// Creates a new ZNavDebugVis object.
    /// </summary>
    /// <returns>Newly created ZNavDebugVis instance.</returns>
    static ZNavDebugVis Create()
    {
        return ZNavDebugVis(new ("ZNavDebugVis"));
    }

    /// <summary>
    /// Assigns a random color translation to this debug visualization instance.
    /// This is done only once so that all spawned visuals share the same color.
    /// </summary>
    play void setPathColor()
    {
        if (hasPathColor) return;

        // Pick a random translation from the predefined list
        PathColor = PathColors[random(0, 6)];
        hasPathColor = true;
    }

    /// <summary>
    /// Adds a visual line between two actors.
    /// </summary>
    /// <param name="start">Actor at the start of the line.</param>
    /// <param name="end">Actor at the end of the line.</param>
    play void AddLine(actor start, actor end)
    {
        setPathColor();

        // Validate actors
        if (!start || !end) return;

        // Spawn line at midpoint between the two actors
        vector3 MidPos = Math.Lerp(start.pos, end.pos, 0.5);
        let visLine = ZNavLineVis(Actor.Spawn("ZNavLineVis", MidPos));

        if (visLine)
        {
            Lines.push(visLine);

            // Rotate sprite to face the start point (plus 90 degrees for sprite alignment)
            visLine.angle = visLine.angleTo(start) + 90.;

            // Adjust roll to match vertical pitch between points
            float ptch = Math.getPitch(visLine.pos, start.pos);
            visLine.A_SetRoll(-ptch, 0, AAPTR_DEFAULT);

            // Scale line length to match distance
            visLine.Scale.X = visLine.Distance3D(start);

            // Apply shared color translation
            visLine.A_SetTranslation(PathColor);
        }
    }

    /// <summary>
    /// Adds a visual line between two world positions.
    /// </summary>
    /// <param name="start">Start position.</param>
    /// <param name="end">End position.</param>
    play void AddLinePos(vector3 start, vector3 end)
    {
        setPathColor();

        // Direction vector used for length calculation
        vector3 diff = end - start;

        // Spawn line at midpoint
        vector3 MidPos = Math.Lerp(start, end, 0.5);
        let visLine = ZNavLineVis(Actor.Spawn("ZNavLineVis", MidPos));

        if (visLine)
        {
            Lines.push(visLine);

            // Angle between two points in world space (plus sprite alignment offset)
            visLine.angle = Math.getAbsAngle(start, end) + 90;

            // Adjust roll for vertical pitch
            float ptch = Math.getPitch(visLine.pos, start);
            visLine.A_SetRoll(-ptch, 0, AAPTR_DEFAULT);

            // Scale to full 3D distance between points
            visLine.Scale.X = diff.length();

            // Apply shared color translation
            visLine.A_SetTranslation(PathColor);
        }
    }

    /// <summary>
    /// Adds a spot icon at the given position.
    /// Typically used to mark path nodes or waypoints.
    /// </summary>
    /// <param name="position">World position of the icon.</param>
    /// <param name="scale">Optional scale multiplier for the icon.</param>
    play void AddSpot(vector3 position, double scale = 1.0)
    {
        setPathColor();

        ZNavSpotIcon pathIcon = ZNavSpotIcon(Actor.Spawn("ZNavSpotIcon", position));
        if (pathIcon)
        {
            Icons.push(pathIcon);

            // Apply shared color and scaling
            pathIcon.A_SetTranslation(PathColor);
            pathIcon.A_SetScale(scale, scale);
        }
    }

    /// <summary>
    /// Connects all spawned spot icons with lines in sequential order.
    /// </summary>
    play void AddLinesToSpots()
    {
        int MaxIcons = Icons.size();

        for (int i = 0; i < MaxIcons; i++)
        {
            int j = i + 1;
            if (j >= MaxIcons) break;

            ZNavSpotIcon last = Icons[i];
            ZNavSpotIcon next = Icons[j];

            AddLine(last, next);
        }
    }

    /// <summary>
    /// Destroys all spawned line and spot actors and clears internal lists.
    /// </summary>
    play void ClearLines()
    {
        // Destroy all line visuals
        for (int i = 0; i < Lines.Size(); i++)
        {
            ZNavLineVis pvline = Lines[i];
            if (pvline) pvline.destroy();
        }

        // Destroy all spot icons
        for (int i = 0; i < Icons.Size(); i++)
        {
            ZNavSpotIcon pathIcon = Icons[i];
            if (pathIcon) pathIcon.destroy();
        }

        Lines.clear();
        Icons.clear();
    }
}

/// <summary>
/// Actor used to visualize a straight path segment as a stretched sprite.
/// </summary>
Class ZNavLineVis : Actor
{
    Default
    {
        +NOCLIP;        // Does not collide with world geometry
        +NOBLOCKMAP;    // Not added to blockmap (no collision checks)
        +NOGRAVITY;     // Not affected by gravity
        +WALLSPRITE;    // Oriented like a wall rather than billboard
        +ROLLSPRITE;    // Supports roll rotation
        +BRIGHT;        // Fullbright (not affected by lighting)
    }

    States
    {
        Spawn:
            ZNAV A 1 Bright;
            ZNAV AAA 800 Bright;
            Stop;
    }
}

/// <summary>
/// Actor used to visualize a waypoint or node along a navigation path.
/// </summary>
Class ZNavSpotIcon : Actor
{
    Default
    {
        +NOCLIP;             // No collision
        +NOBLOCKMAP;         // Not in blockmap
        +NOGRAVITY;          // Floats in place
        +BRIGHT;             // Fullbright
        +FORCEXYBILLBOARD;   // Always faces camera on XY plane
        Scale 0.5;           // Default icon size
    }

    States
    {
        Spawn:
            ZNAV B 1 Bright;
            ZNAV BBB 800 Bright;
            Stop;
    }
}
