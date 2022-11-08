/* 
JSON Schema - this explains the contents of /ZJSON JSON files
{
    vertices: [], // an array of floats ordered in groups of three, x,y,z
    nodes: [ //a list of polygons with n sides that defines a walkable region
        {
            c: [x,y,z], // the centroid of the polygon
            n: [ neighborID ], //list of indices of polygons which share edges with this polygon
            v: [ vertexIDs ], //a list of indices which references a block of 3 doubles in the vertex array above,
            p: [ [vertexID, vertexID] ], //portals, a list of vertex pairs which define a shared  line segment of the polygon with an adjacent polygon
            f: integer, //flags denoting special properties of a node
            t: integer // reference to a thing tag, for stuff like leap nodes
        }
    ] 
}

to generate a JSON pathfinding schema for sunset, you will need to 
first create a navigation mesh for your level. this can be accomplished by
exporting your level as an OBJ, importing into blender and using the nav mesh tool
to generate a nav mesh.

the method `createZone`
https://github.com/donmccurdy/three-pathfinding/blob/6691f83044106ba63b715f8accbce4ea6fb99cec/src/Pathfinding.js#L26
will create an object structured similarly to the json schema above
*/


class ZNavParser : ZNav
{
    static play bool CheckForZNavMesh( string mapname )
    {
        string jsonName = string.format('models/nav/%s.json', mapname);
        int lump = Wads.CheckNumForFullName(jsonName);

        if(lump==-1)
        {
            return false;
		}
        return true;
    }

    static play ZNavMesh BuildGraph ( string mapname, ZNavThinker LevelNavigator)
    {
        string jsonName = string.format('models/nav/%s.json', mapname);
        int lump = Wads.CheckNumForFullName(jsonName);

        JsonElementOrError data = JSON.parse( Wads.ReadLump( lump ), false );

        if ( data is "JsonError" )
        {
			console.printf("%s: could not read file", jsonName);
            return null;
		}

        JSONElement elem = JSONElement ( data );
        JSONObject obj = JSONObject( elem );

        return ZNavParser.CreateZNavMesh( obj, LevelNavigator );
    }   

    static play ZNavMesh CreateZNavMesh( JSONObject obj, ZNavThinker LevelNavigator )
    {
        ZNavMesh ZNavMesh = new('ZNavMesh');
        ZNavMesh.init(LevelNavigator);

        /* 
            create cell space partitioning 
        */

        JSONInt jlengthInt = JSONInt( obj.get( "length" ) );
        int MaxCells = jlengthInt.i;
        ZNavMesh.MaxCells = MaxCells;
        for(int j = 0; j < MaxCells; j++)
        {
            ZNavCell cell = ZNavParser.CreateZNavCell( j );
            ZNavMesh.cells.push( cell );
        }

        // grid offset
        JSONInt joriginx = JSONInt( obj.get( "originx" ) );
        JSONInt joriginy = JSONInt( obj.get( "originy" ) );
        ZNavMesh.GridOrigin = ( joriginx.i, joriginy.i );

        //grid columns and rows
        JSONInt jsizeX = JSONInt( obj.get( "sizex" ) );
        JSONInt jsizeY = JSONInt( obj.get( "sizey" ) );
        ZNavMesh.GridSize = ( jsizeX.i, jsizeY.i );

        //grid columns and rows
        JSONInt jRes = JSONInt( obj.get( "res" ) );
        ZNavMesh.GridRes = jRes.i;

        /* 
            serialize vertex data 
        */
        JsonElement vertexElem = obj.get( "vertices" );
        JsonArray vertexArr = JsonArray ( vertexElem );

        uint n = vertexArr.size();
        for( uint i = 0; i < n; i++ )
        {
            JSONInt jvertval = JSONInt( vertexArr.get(i) );
            ZNavMesh.vertices.push( jvertval.i );
        }

        /* 
            serialize group data 
        */
        JSONInt jGrpCount = JSONInt ( obj.get( "groups" ) );
        int grpcount = jGrpCount.i;

        for( int j = 0; j < grpcount; j++ )
        {
            ZNavGroup group = ZNavParser.CreateZNavGroup(ZNavMesh, j);
            ZNavMesh.groups.push( group );
        }

        /* 
            serialize node data 
        */

        JsonArray nodesArr = JsonArray ( obj.get( "nodes" ) );
        n = nodesArr.size();

        for( uint i = 0; i < n; i++ )
        {
            JSONObject jnodeObj = JSONObject( nodesArr.get(i) );
            ZNavNode node = ZNavParser.CreateZNavNode( ZNavMesh, jnodeObj, i );
        }

        return ZNavMesh;
    }

    static play ZNavNode CreateZNavNode ( ZNavMesh mesh, jsonobject j_obj, int nodeID  )
    {
        ZNavNode node = new('ZNavNode');

        /* parent */
        node.mesh = mesh;

         /* set ID */
        node.nodeID = nodeID;

        /* reusable bean counter */
        uint n;

        /* set groupID */
        JSONInt jsonGroupID = JSONInt( j_obj.get('g') );
        node.group = mesh.groups[ jsonGroupID.i ];
        //node.groupID = group.groupID;
        node.group.nodes.push( node );

        /* add vertices */
        JsonArray vertexArr = JsonArray( j_obj.get('v') );
        n = vertexArr.size();
        for (uint i = 0; i < n; i++)
        {
            JSONInt jvtxID = JSONInt( vertexArr.get(i) );
            node.vertexIDs.push ( jvtxID.i );
        }

        /* set centroid */
        JsonArray centroidElem = JsonArray(j_obj.get('c'));
        JSONInt jcx = JSONInt( centroidElem.get(0) );
        JSONInt jcy = JSONInt( centroidElem.get(1) );
        JSONInt jcz = JSONInt( centroidElem.get(2) );
        node.centroid = ( jcx.i, jcy.i, jcz.i );

        /* set portals */
        JsonArray portalsArr = JsonArray( j_obj.get('p') );
        n = portalsArr.size();
        for (uint i = 0; i < n; i++)
        {
            JsonArray portalArr = JsonArray( portalsArr.get(i) );
            ZNavPortal portal = ZNavParser.CreateZNavPortal(portalArr);
            node.portals.push(portal);
        }

        /* set neighbors */
        JsonArray neighborArr = JsonArray( j_obj.get('n') );
        n = neighborArr.size();
        for (uint i = 0; i < n; i++)
        {
            JSONInt jneighborID = JSONInt( neighborArr.get(i) );
            node.neighborIDs.push ( jneighborID.i );
        }
        node.MaxNeighborIDs = node.neighborIDs.size();

        /* add cell space */
        JsonArray jcellsArr = JsonArray(j_obj.get('b'));
        n = jcellsArr.size();
        for (uint i = 0; i < n; i++)
        {
            JSONInt jCellIndex = JSONInt( jcellsArr.get( i ) );
            mesh.cells[ jCellIndex.i ].addNode( node );
        }

        node.cell = mesh.getCellFromPosition (node.centroid);

        /* set flags */
        node.flags = 0;
        int optFlags = ZNavParser.getOptionalInt( j_obj, 'f');
        if ( optFlags > 0) node.flags = optFlags;

        /* set helpers */
        int optHelperTID = ZNavParser.getOptionalInt( j_obj, 'h');
        if ( optHelperTID > 0) {
            node.helperTID = optHelperTID;
            node.getHelper(node.helperTID);
        }

        node.clear();
        mesh.nodes.push( node );

        return node;
    }

    static int getOptionalInt ( JSONObject jObj, string key)
    {
        JSONElement jElem = JSONElement(jObj.get(key));

        if (!jElem) return -1;

        JSONInt jInt = JSONInt( jElem );

        return jInt.i;
    }

    static ZNavGroup CreateZNavGroup(ZNavMesh mesh, int groupID)
    {
        ZNavGroup grp = new('ZNavGroup');
        grp.mesh = mesh;
        grp.groupID = groupID;
        return grp;
    }

    static ZNavCell CreateZNavCell(int id)
    {
        ZNavCell cell = new('ZNavCell');
        cell.id = id;
        return cell;
    }

    static ZNavPortal CreateZNavPortal (JsonArray portalArr )
    {
        ZNavPortal portal = new ('ZNavPortal');
        uint n = portalArr.size();
        for (uint i = 0; i < n; i++)
        {
            JSONInt portalID = JSONInt(portalArr.get(i));
            portal.vertexIDs.push( portalID.i );
        }
        return portal;
    }

}
