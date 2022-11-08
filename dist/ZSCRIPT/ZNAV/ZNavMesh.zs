/*
    This is a direct adaptation of three-pathfinding:
    https://github.com/donmccurdy/three-pathfinding
*/
class ZNav abstract {}

/*
    cell space partitioning adapted from
    https://commons.apache.org/proper/commons-math/javadocs/api-3.6.1/org/apache/commons/math3/geometry/partitioning/BSPTree.html
*/
Class ZNavCell : ZNav
{
    array<ZNavNode>nodes;
    array<ZNavAgent>agents;
    int id;
    int cost;

    void addNode ( ZNavNode node )
    {
        self.nodes.push(node);
    }

    void addAgent( ZNavAgent agent )
    {
        cost += agent.radius;
        self.agents.push( agent );
    }

    void removeAgent( ZNavAgent agent )
    {
        int index = agents.find(agent);
        if (index != agents.size())
        {
            cost -= agent.radius;
            agents.delete(index, 1);
        }
    }
}

Class ZNavMesh : ZNav
{
    /* master controller */
    ZNavThinker master;

    /* members */
    array<double>vertices;
    array<ZNavGroup> groups;
    array<ZNavNode> nodes;
    ZNavChannel channel;
    Triangle tri;

    /* 
        cell space partitioning 
    */
    array<ZNavCell>cells;
    int MaxCells;
    vector2 GridOrigin;
    vector2 GridSize;
    int GridRes;

    void init(ZNavThinker LevelNavigator)
    {
        self.master = LevelNavigator;
        self.tri = new ('Triangle');
        self.channel = ZNavChannel.Create();
    }

    static bool isPointInNode(vector3 pt, ZNavNode node)
    {
        if (!node) return false;
		int i = -1;
		bool c = false;
		int l = node.vertexIDs.size();
		int j = l -1;
		bool q;

        for (c = false, i = -1, l = node.vertexIDs.size(), j = l-1; ++i < l; j = i)
		{
			vector3 vtxA = node.getVertexByIndex( i );
			vector3 vtxB = node.getVertexByIndex( j );

			q = ((vtxA.y <= pt.y && pt.y < vtxB.y) || (vtxB.y <= pt.y && pt.y < vtxA.y)) && 
			(pt.x < (vtxB.x - vtxA.x) * (pt.y - vtxA.y) / (vtxB.y - vtxA.y) + vtxA.x) && (c = !c);
		}

		return c;
    }

    static bool isVectorInNode(vector3 pt, ZNavNode node)
    {
        double MinZ = Int.Max;
        double MaxZ = -Int.MAX;

        for ( int i = 0; i < node.vertexIDs.size(); i++ )
        {
            vector3 vtx = node.getVertexByIndex(i);
            MinZ = min (vtx.z, MinZ);
            MaxZ = max (vtx.z, MaxZ);
        }

        if ( pt.z < MinZ - 8 || pt.z > MaxZ + 8) return false;
        return ZNavMesh.isPointInNode( pt, node);
    }

    static ZNavPortal getPortalsFromTo( ZNavNode a, ZNavNode b)
    {
        for (int i = 0; i < a.neighborIDs.size(); i++ )
        {
            if ( a.neighborids[i] == b.nodeID )
            {
                return a.portals[i];
            }
        }
        return null;
    }

    static ZNavNode getClosestNodeIn (vector3 pos, in array<ZNavNode>nodes, bool usePoly = false)
    {
        if ( !nodes.size() ) return null;

        int closestNode;
        int closestDistance = Int.MAX;

        int closestInside = -1;
        int closestDistanceInside = Int.MAX;

        for (int i = 0; i < nodes.size(); i++)
        {

            if ( nodes[i].flags & NAV_ARC )
            {
                continue;
            }
            
            if ( nodes[i].flags & NAV_LEAP )
            {
                continue;
            }

            if ( nodes[i].flags & NAV_LAND )
            {
                continue;
            }

            int distance = Math.distanceToSquared( pos, nodes[i].centroid );

            if (distance < closestDistance)
            {
                closestNode = i;
                closestDistance = distance;
            }

            if (usePoly)
            {
                if ( (distance < closestDistanceInside) && ZNavMesh.isVectorInNode(pos, nodes[i]) )
                {
                    closestInside = i;
                    closestDistanceInside = distance;
                }
            }
        }

        if ( closestInside > -1 )
        {
            return nodes[closestInside];
        }

        return nodes[closestNode];
    }

    int getIndexFromPosition( vector3 pos )
    {
        vector2 gridPos = ( 
            floor( ( pos.x - self.GridOrigin.x ) / self.GridRes ), 
            floor( ( self.GridOrigin.y - pos.y ) / self.GridRes )
        );

        return (( GridSize.x + 1 ) * gridPos.y) + gridPos.x;
    }

    ZNavCell getCellFromPosition( vector3 pos )
    {
        int index = self.getIndexFromPosition(pos);
        if (index < 0 || index >= MaxCells ) return null;
        return cells[index];
    }

    void UpdateCell (int lastIndex, int index, ZNavAgent agent)
    {
        if (lastIndex > -1 && lastIndex < MaxCells )
        {
            cells[lastIndex].removeAgent(agent);
        }

        if (index > -1 && index < MaxCells )
        {
            cells[index].addAgent(agent);
        }
    }

    vector3 getVertex (int vertexID)
    {
        vertexID = vertexID * 3;
        return ( self.vertices[vertexID], self.vertices[vertexID + 1], self.vertices[vertexID + 2] );
    }

    ZNavNode getNodeByID (int nodeID)
    {
        return nodes[nodeID];
    }

    /* 
        pathfinding 
    */
    play bool FindPath(int GroupID, vector3 startPos, vector3 endPos, ZNavRoute route)
    {

        ZNavNode startNode = getClosestNodeInGroup ( startPos, GroupID );
        ZNavNode endNode = getClosestNodeInGroup ( endPos, GroupID );

        if (startNode == endNode)
        {
            route.clear();
            route.push( startPos );
            route.push( endPos );
            return true;
        }

        ZNavGroup group = getGroupByID(GroupID);

        array<ZNavNode>foundNodes;
        bool foundPath = ZNavAStar.Search( group, startNode, endNode, foundNodes);

        channel.clear();
        channel.addSingle ( startPos);

        for (int i = 0; i < foundNodes.size(); i++)
        {
            int j = i + 1;
            if ( j < foundNodes.size() )
            {
                ZNavNode polygon = foundNodes[i];

                if (polygon.flags)
                {
                    endPos = polygon.centroid;
                    break;
                }

                ZNavNode nextPolygon = foundNodes[j];
                ZNavPortal portal = ZNavMesh.getPortalsFromTo(polygon, nextPolygon);
                channel.addPair( 
                    getVertex(portal.vertexIds[0]), 
                    getVertex(portal.vertexIds[1])
                );
            }
        }

        channel.addSingle ( endPos );
        channel.stringPull( route );

        return foundPath;
    }

    int getNearestGroupID ( vector3 pos, bool usepoly = false )
    {
        ZNavCell cell = getCellFromPosition( pos );
        ZNavNode node;

        if ( cell != null && cell.nodes.size() > 0)
        {
            node = ZNavMesh.getClosestNodeIn ( pos, cell.nodes, usepoly );
        } else {
            node = ZNavMesh.getClosestNodeIn ( pos, self.nodes, usepoly );
        }

        if (node) return node.group.groupID;
        return -1;
    }

    ZNavGroup getGroupByID ( int groupID )
    {
        return self.groups[groupID];
    }

    ZNavGroup getNearestGroupTo ( vector3 pos )
    {
        int groupID = getNearestGroupID (pos);
        return groups[groupID];
    }    

    ZNavNode getClosestNodeInGroup( vector3 pos, int groupID )
    {
        return ZNavMesh.getClosestNodeIn (pos, groups[groupID].nodes, false) ; 
    }

    ZNavNode getClosestNodeTo ( vector3 pos )
    {
        return ZNavMesh.getClosestNodeIn (pos, self.nodes, false) ; 
    }

    ZNavNode getClosestNodeEx ( vector3 pos )
    {
        return ZNavMesh.getClosestNodeIn (pos, self.nodes, true) ; 
    }

    void addNodeToDepthList (int nodeID, int depth, in out array<int>list )
    {
        list.push(nodeID);
        list.push(depth);
    }

    void addDepthToDepthList (int nodeID, int depth, in out array<int>list )
    {
        for (int i = 0; i < list.size()  / 2; i++)
        {
            int idIndex = i * 2;
            if ( nodeID == list[idIndex] )
            {
                list[idIndex+1] = depth;
                break;
            }
        }
    }

    int getDepthFromList (int nodeID, in out array<int>list )
    {
        for (int i =0; i < list.size() / 2; i++)
        {
            int idIndex = i * 2;
            if ( nodeID == list[idIndex] )
            {
                return list[idIndex+1];
            }
        }

        return -1;
    }

    bool depthListHasMember ( int nodeID, in out array<int>list)
    {
        for (int i =0; i < list.size() / 2; i++)
        {
            int idIndex = i * 2;
            if ( nodeID == list[idIndex] )
            {
                return true;
            }
        }

        return false;
    }

    vector3, ZNavNode clampStep( vector3 startRef, vector3 endRef, ZNavNode node )
    {
        array<ZNavNode>queue;
        array<int>nodeDepth;

        vector3 point;
        vector3 endPoint;

        point = endRef; 
        endPoint = point;

        ZNavNode closestNode;
        vector3 closestPoint;
        double closestDistance = INT.Max;

        queue.push(node);
        addNodeToDepthList (node.nodeID, 0, nodeDepth);

        int runaway = 0;

        while ( queue.size() )
        {

            runaway++;
            if (runaway > 2000) 
            {
                break;
            }

            ZNavNode currentNode = queue[queue.size() - 1];
            queue.delete(queue.size() - 1, 1);

            tri.set (
                currentNode.getVertexByIndex(0),
                currentNode.getVertexByIndex(1),
                currentNode.getVertexByIndex(2)
            );

            point = tri.closestPointToPoint(endPoint);
            double dist = Math.distanceToSquared( point, endPoint );

            if ( dist < closestDistance ) 
            {
				closestNode = currentNode;
				closestPoint = point;
				closestDistance = dist;
			}

            int depth = getDepthFromList( currentNode.nodeID, nodeDepth );
            if (depth > 2) continue;

            for (int i = 0; i < currentNode.neighborIDs.size(); i++ )
            {
                ZNavNode neighbor = currentNode.getNeighborByIndex( i );
                if ( depthListHasMember ( neighbor.nodeID, nodeDepth ) )
                {
                    continue;
                } else {
                    addNodeToDepthList ( neighbor.nodeID, depth+1, nodeDepth);
                    queue.push ( neighbor );
                }
            }
        }

        return closestPoint, closestNode;

    }

}

Class ZNavGroup : ZNav
{
    int groupID;
    ZNavMesh mesh;
    array<ZNavNode>nodes;
    
    ZNavNode getClosestNode ( vector3 pos )
    {
        return mesh.getClosestNodeInGroup ( pos, self.groupID );
    }

    void resetNodes()
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            self.nodes[i].clear();
        }
    }
}

Class ZNavNode : ZNav
{
    // network
    ZNavGroup group;
    ZNavMesh mesh;

    //data
    NavHelper helper;
    int helperTID;

    int flags;
    int nodeID;
    vector3 centroid;

    // components
    int MaxNeighborIDs;
    array<Int>neighborIDs;
    array<Int>vertexIDs;
    array<ZNavPortal> portals;
    bool isLeap, isArc, isLanding;
    ZNavCell cell;
    
    /* AStar Stuff */
    ZNavNode parent;
    bool closed;
    bool visited;
    int cost;
    int f;
    int g;
    int h;

    play void getHelper ( int tid )
    {
        ThinkerIterator it = ThinkerIterator.Create ('NavHelper');
        NavHelper nt;

        while (nt = NavHelper(it.Next())) 
        {
            if ( nt && nt.tid == tid )
            {
                helper = nt;
                break;
            }
        }

        if (!helper)
        {
            console.printf( "\c[red]couldn\'t find helper node\cl" );
        } else {
            helper.addNode(self);
        }
    }

    play vector3 pointOnPerimter( vector3 pt )
    {
        double closestDistance = Int.Max;
        vector2 closestPoint;

        int size = self.vertexIDs.size();

        for ( int i = 0; i < size; i++ )
        {
            int j = i + 1;
            if ( j >= size ) j = 0;

            vector3 a = getVertexByIndex(i);
            vector3 b = getVertexByIndex(j);

            vector2 c = Math.getClosestPointOnLine ( a, b, pt );
            double dist = Math.distanceToSquared2( pt, (c, 0) );

            if ( dist < closestDistance )
            {
                closestDistance = dist;
                closestPoint = c;
            }
        }

        return (closestPoint, pt.z);
    }

    play vector3 constrainPoint ( vector3 pt )
    {
        vector3 nextPt = pt;

        if ( !ZNavMesh.isPointInNode(pt, self) )
        {
            nextPt = self.pointOnPerimter(pt);
        }
        return nextPt;
    }

    int getCost()
    {
        return cost;
    }

    vector3 getVertex (int vertexID)
    {
        return self.mesh.getVertex( vertexID );
    }

    vector3 getVertexByIndex (int index)
    {
        return self.mesh.getVertex( vertexIDs[index] );
    }

    ZNavNode getNeighbor( int neighborID )
    {
        return self.mesh.nodes [ neighborID ];
    }

    ZNavNode getNeighborByIndex ( int index )
    {
        return self.mesh.nodes [ self.neighborIDs[ index ] ];
    }

    void clear()
    {
        closed = false;
        visited = false;
        parent = null;
        cost = 1;
        f = 0;
        g = 0;
        h = 0;
    }

    vector3 getNearestPointTo( vector3 pos )
    {
        mesh.tri.set ( getVertexByIndex(0), getVertexByIndex(1), getVertexByIndex(2) );
        return mesh.tri.closestPointToPoint ( pos );
    }
}
