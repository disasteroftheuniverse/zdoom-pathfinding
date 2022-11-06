class ZLevelHandler : StaticEventHandler abstract
{
    
	static void emit(string name = "", int arg1 = 0, int arg2 = 0, int arg3 = 0) {
		EventHandler.SendNetworkEvent(name, arg1, arg2, arg3);
	}

    virtual void onEnter (WorldEvent e){}

    virtual void onReEnter (WorldEvent e){}

    override void WorldLoaded  (WorldEvent e)
    {
        if (!e.isSaveGame && !e.isReOpen)
        {
            onEnter(e);
            return;
        }

        if (e.isReOpen)
        {
            onReEnter(e);
            return;
        }
    }
}

Class ZNavHandler : ZLevelHandler
{
    override void onEnter(WorldEvent e)
    {
        bool hasNavGraph = NavParser.CheckForNavMesh( Level.MapName );
        if ( hasNavGraph )
        {
            ZNavThinker ZLevelNavigator = new('ZNavThinker');
            ZLevelNavigator.init( Level.MapName );
        }
    }
}
