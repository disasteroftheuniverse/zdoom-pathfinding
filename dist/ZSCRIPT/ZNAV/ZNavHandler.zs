/// <summary>
/// Handles level-related events and provides static event emission functionality.
/// Inherits from StaticEventHandler and is abstract.
/// </summary>
class ZLevelHandler : StaticEventHandler abstract
{
    /// <summary>
    /// Emits a network event with optional arguments.
    /// </summary>
    /// <param name="name">Name of the event to emit (default is empty string).</param>
    /// <param name="arg1">First integer argument (default 0).</param>
    /// <param name="arg2">Second integer argument (default 0).</param>
    /// <param name="arg3">Third integer argument (default 0).</param>
	static void emit(string name = "", int arg1 = 0, int arg2 = 0, int arg3 = 0) {
		EventHandler.SendNetworkEvent(name, arg1, arg2, arg3);
	}

    /// <summary>
    /// Called when a level is first entered. Can be overridden by subclasses.
    /// </summary>
    /// <param name="e">WorldEvent information for the level entry.</param>
    virtual void onEnter (WorldEvent e){}

    /// <summary>
    /// Called when a level is re-entered (reopened). Can be overridden by subclasses.
    /// </summary>
    /// <param name="e">WorldEvent information for the re-entry.</param>
    virtual void onReEnter (WorldEvent e){}

    /// <summary>
    /// Handles the WorldLoaded event.
    /// Invokes onEnter or onReEnter depending on whether the level is being opened for the first time or reopened.
    /// </summary>
    /// <param name="e">WorldEvent information for the level loading.</param>
    override void WorldLoaded  (WorldEvent e)
    {
        if (!e.isSaveGame && !e.isReOpen)
        {
            // Level is being loaded for the first time
            onEnter(e);
            return;
        }

        if (e.isReOpen)
        {
            // Level is being reopened
            onReEnter(e);
            return;
        }
    }
}

/// <summary>
/// Specific handler for navigation-related logic in a level.
/// Inherits from ZLevelHandler.
/// </summary>
Class ZNavHandler : ZLevelHandler
{
    /// <summary>
    /// Called when a level is entered. Checks for a navigation mesh and initializes the navigator if one exists.
    /// </summary>
    /// <param name="e">WorldEvent information for the level entry.</param>
    override void onEnter(WorldEvent e)
    {
        // just check to see if this level has a nav mesh
        bool hasNavGraph = ZNavParser.CheckForNavMesh( Level.MapName );

        if ( hasNavGraph )
        {
            // Initialize a ZNavThinker to manage navigation
            ZNavThinker ZLevelNavigator = new('ZNavThinker');
            ZLevelNavigator.init( Level.MapName );
        }
    }
}
