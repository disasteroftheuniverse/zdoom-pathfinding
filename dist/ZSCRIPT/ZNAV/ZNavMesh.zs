/// <summary>
/// Base navigation class.
/// All navigation-related classes derive from this.
/// </summary>
/// <remarks>
/// Direct adaptation of three-pathfinding:
/// https://github.com/donmccurdy/three-pathfinding
/// </remarks>
class ZNav abstract {}

/// <summary>
/// Spatial partition cell used for fast spatial queries.
/// Works similarly to Doom's blockmap: nodes and agents are assigned to cells
/// so nearby lookups only check local data instead of the full mesh.
/// </summary>
Class ZNavCell : ZNav
{
    /// <summary>Navigation nodes that intersect this cell.</summary>
    array<ZNavNode> nodes;

    /// <summary>Agents currently inside this cell.</summary>
    array<ZNavAgent> agents;

    /// <summary>Cell identifier.</summary>
    int id;

    /// <summary>
    /// Accumulated traversal cost based on agents occupying the cell.
    /// Used to discourage crowding.
    /// </summary>
    int cost;

    /// <summary>
    /// Adds a navigation node reference to this cell.
    /// </summary>
    void addNode(ZNavNode node)
    {
        self.nodes.push(node);
    }

    /// <summary>
    /// Adds an agent to the cell and increases traversal cost.
    /// </summary>
    void addAgent(ZNavAgent agent)
    {
        cost += agent.radius;
        self.agents.push(agent);
    }

    /// <summary>
    /// Removes an agent from the cell and reduces traversal cost.
    /// </summary>
    void removeAgent(ZNavAgent agent)
    {
        int index = agents.find(agent);
        if (index != agents.size())
        {
            cost -= agent.radius;
            agents.delete(index, 1);
        }
    }
}

/// <summary>
/// Navigation mesh containing all nodes, groups, and spatial partitioning data.
/// Provides pathfinding and spatial lookup utilities.
/// </summary>
Class ZNavMesh : ZNav
{
    /// <summary>Master controller handling navigation updates.</summary>
    ZNavThinker master;

    /* ---------- Mesh Data ---------- */

    /// <summary>Flat array of mesh vertices (XYZ triplets).</summary>
    array<double> vertices;

    /// <summary>Connected components of nodes.</summary>
    array<ZNavGroup> groups;

    /// <summary>All nodes in the mesh.</summary>
    array<ZNavNode> nodes;

    /// <summary>Funnel channel used to smooth final paths.</summary>
    ZNavChannel channel;

    /// <summary>Triangle helper for closest-point calculations.</summary>
    Triangle tri;

    /* ---------- Cell Space Partitioning ---------- */

    /// <summary>Spatial grid cells.</summary>
    array<ZNavCell> cells;

    /// <summary>Total number of cells.</summary>
    int MaxCells;

    /// <summary>World-space origin of the grid.</summary>
    vector2 GridOrigin;

    /// <summary>Dimensions of the grid in cells.</summary>
    vector2 GridSize;

    /// <summary>Cell resolution (world units per cell).</summary>
    int GridRes;

    /* ---------- Collision Helpers ---------- */

    /// <summary>Bounding box for source agent.</summary>
    ZNavAABB3 bboxSrc;

    /// <summary>Bounding box for target area.</summary>
    ZNavAABB3 bboxTarg;

    /// <summary>
    /// Initializes runtime helpers for the navigation mesh.
    /// </summary>
    void init(ZNavThinker LevelNavigator)
    {
        self.master = LevelNavigator;
        self.tri = new('Triangle');
        self.channel = ZNavChannel.Create();
    }

    /// <summary>
    /// Tests whether a 2D point lies inside a polygon node using ray casting.
    /// Z is ignored.
    /// </summary>
    static bool isPointInNode(vector3 pt, ZNavNode node)
    {
        if (!node) return false;

        int i = -1;
        bool c = false;
        int l = node.vertexIDs.size();
        int j = l - 1;
        bool q;

        for (c = false, i = -1, l = node.vertexIDs.size(), j = l - 1; ++i < l; j = i)
        {
            vector3 vtxA = node.getVertexByIndex(i);
            vector3 vtxB = node.getVertexByIndex(j);

            q = ((vtxA.y <= pt.y && pt.y < vtxB.y) || (vtxB.y <= pt.y && pt.y < vtxA.y)) &&
                (pt.x < (vtxB.x - vtxA.x) * (pt.y - vtxA.y) / (vtxB.y - vtxA.y) + vtxA.x) &&
                (c = !c);
        }

        return c;
    }

    /// <summary>
    /// Tests whether a 3D point lies within a node, including Z tolerance.
    /// </summary>
    static bool isVectorInNode(vector3 pt, ZNavNode node)
    {
        double MinZ = Int.Max;
        double MaxZ = -Int.MAX;

        for (int i = 0; i < node.vertexIDs.size(); i++)
        {
            vector3 vtx = node.getVertexByIndex(i);
            MinZ = min(vtx.z, MinZ);
            MaxZ = max(vtx.z, MaxZ);
        }

        // Allow small vertical tolerance
        if (pt.z < MinZ - 8 || pt.z > MaxZ + 8) return false;

        return ZNavMesh.isPointInNode(pt, node);
    }

    /// <summary>
    /// Returns the portal connecting two adjacent nodes.
    /// </summary>
    static ZNavPortal getPortalsFromTo(ZNavNode a, ZNavNode b)
    {
        for (int i = 0; i < a.neighborIDs.size(); i++)
        {
            if (a.neighborids[i] == b.nodeID)
            {
                return a.portals[i];
            }
        }
        return null;
    }

    /// <summary>
    /// Finds the closest node to a position within a given node list.
    /// Optionally requires the point to lie inside the polygon.
    /// </summary>
    static ZNavNode getClosestNodeIn(vector3 pos, in array<ZNavNode> nodes, bool usePoly = false)
    {
        if (!nodes.size()) return null;

        int closestNode;
        int closestDistance = Int.MAX;

        int closestInside = -1;
        int closestDistanceInside = Int.MAX;

        for (int i = 0; i < nodes.size(); i++)
        {
            // Skip special traversal nodes
            if (nodes[i].flags & NAV_ARC)  continue;
            if (nodes[i].flags & NAV_LEAP) continue;
            if (nodes[i].flags & NAV_LAND) continue;

            int distance = Math.distanceToSquared(pos, nodes[i].centroid);

            if (distance < closestDistance)
            {
                closestNode = i;
                closestDistance = distance;
            }

            if (usePoly)
            {
                if ((distance < closestDistanceInside) &&
                    ZNavMesh.isVectorInNode(pos, nodes[i]))
                {
                    closestInside = i;
                    closestDistanceInside = distance;
                }
            }
        }

        if (closestInside > -1)
        {
            return nodes[closestInside];
        }

        return nodes[closestNode];
    }

    /// <summary>
    /// Converts a world position to a grid cell index.
    /// </summary>
    int getIndexFromPosition(vector3 pos)
    {
        vector2 gridPos = (
            floor((pos.x - self.GridOrigin.x) / self.GridRes),
            floor((self.GridOrigin.y - pos.y) / self.GridRes)
        );

        return ((GridSize.x + 1) * gridPos.y) + gridPos.x;
    }

    /// <summary>
    /// Returns the spatial cell containing the given position.
    /// </summary>
    ZNavCell getCellFromPosition(vector3 pos)
    {
        int index = self.getIndexFromPosition(pos);
        if (index < 0 || index >= MaxCells) return null;
        return cells[index];
    }

    /// <summary>
    /// Updates agent membership when moving between cells.
    /// </summary>
    void UpdateCell(int lastIndex, int index, ZNavAgent agent)
    {
        if (lastIndex > -1 && lastIndex < MaxCells)
        {
            cells[lastIndex].removeAgent(agent);
        }

        if (index > -1 && index < MaxCells)
        {
            cells[index].addAgent(agent);
        }
    }

    /// <summary>
    /// Returns a vertex position by vertex ID.
    /// </summary>
    vector3 getVertex(int vertexID)
    {
        vertexID = vertexID * 3;
        return (self.vertices[vertexID], self.vertices[vertexID + 1], self.vertices[vertexID + 2]);
    }

    /// <summary>
    /// Returns a node by its node ID.
    /// </summary>
    ZNavNode getNodeByID(int nodeID)
    {
        return nodes[nodeID];
    }

    /* ---------- Pathfinding ---------- */

    /// <summary>
    /// Finds a smoothed path between two positions within a navigation group.
    /// </summary>
    /// <param name="GroupID">Navigation group ID.</param>
    /// <param name="startPos">Starting world position.</param>
    /// <param name="endPos">Target world position.</param>
    /// <param name="route">Output route containing final waypoints.</param>
    /// <returns>True if a path was found.</returns>
    play bool FindPath(int GroupID, vector3 startPos, vector3 endPos, ZNavRoute route)
    {
        ZNavNode startNode = getClosestNodeInGroup(startPos, GroupID);
        ZNavNode endNode = getClosestNodeInGroup(endPos, GroupID);

        // Direct path if both points are in the same polygon
        if (startNode == endNode)
        {
            route.clear();
            route.push(startPos);
            route.push(endPos);
            return true;
        }

        ZNavGroup group = getGroupByID(GroupID);

        array<ZNavNode> foundNodes;
        bool foundPath = ZNavAStar.Search(group, startNode, endNode, foundNodes);

        // Build funnel channel from portals
        channel.clear();
        channel.addSingle(startPos);

        for (int i = 0; i < foundNodes.size(); i++)
        {
            int j = i + 1;
            if (j < foundNodes.size())
            {
                ZNavNode polygon = foundNodes[i];

                // Stop early for special nodes
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

        channel.addSingle(endPos);

        // Funnel algorithm produces final smoothed route
        channel.stringPull(route);

        return foundPath;
    }

    /// <summary>
    /// Returns the nearest group ID to a position.
    /// </summary>
    int getNearestGroupID(vector3 pos, bool usepoly = false)
    {
        ZNavCell cell = getCellFromPosition(pos);
        ZNavNode node;

        if (cell != null && cell.nodes.size() > 0)
        {
            node = ZNavMesh.getClosestNodeIn(pos, cell.nodes, usepoly);
        }
        else
        {
            node = ZNavMesh.getClosestNodeIn(pos, self.nodes, usepoly);
        }

        if (node) return node.group.groupID;
        return -1;
    }

    /// <summary>
    /// Returns a group by ID.
    /// </summary>
    ZNavGroup getGroupByID(int groupID)
    {
        return self.groups[groupID];
    }

    /// <summary>
    /// Returns the nearest group to a world position.
    /// </summary>
    ZNavGroup getNearestGroupTo(vector3 pos)
    {
        int groupID = getNearestGroupID(pos);
        return groups[groupID];
    }

    /// <summary>
    /// Returns the closest node within a group.
    /// </summary>
    ZNavNode getClosestNodeInGroup(vector3 pos, int groupID)
    {
        return ZNavMesh.getClosestNodeIn(pos, groups[groupID].nodes, false);
    }

    /// <summary>
    /// Returns the closest node in the entire mesh.
    /// </summary>
    ZNavNode getClosestNodeTo(vector3 pos)
    {
        return ZNavMesh.getClosestNodeIn(pos, self.nodes, false);
    }

    /// <summary>
    /// Returns closest node, preferring nodes that contain the point.
    /// </summary>
    ZNavNode getClosestNodeEx(vector3 pos)
    {
        return ZNavMesh.getClosestNodeIn(pos, self.nodes, true);
    }

    /* ---------- Utility for BFS Depth Tracking ---------- */

    void addNodeToDepthList(int nodeID, int depth, in out array<int> list)
    {
        list.push(nodeID);
        list.push(depth);
    }

    void addDepthToDepthList(int nodeID, int depth, in out array<int> list)
    {
        for (int i = 0; i < list.size() / 2; i++)
        {
            int idIndex = i * 2;
            if (nodeID == list[idIndex])
            {
                list[idIndex + 1] = depth;
                break;
            }
        }
    }

    int getDepthFromList(int nodeID, in out array<int> list)
    {
        for (int i = 0; i < list.size() / 2; i++)
        {
            int idIndex = i * 2;
            if (nodeID == list[idIndex])
            {
                return list[idIndex + 1];
            }
        }
        return -1;
    }

    bool depthListHasMember(int nodeID, in out array<int> list)
    {
        for (int i = 0; i < list.size() / 2; i++)
        {
            int idIndex = i * 2;
            if (nodeID == list[idIndex])
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Clamps a movement step to the nearest reachable point on connected polygons.
    /// Searches nearby nodes up to limited depth.
    /// </summary>
    vector3, ZNavNode clampStep(vector3 startRef, vector3 endRef, ZNavNode node)
    {
        array<ZNavNode> queue;
        array<int> nodeDepth;

        vector3 point;
        vector3 endPoint = endRef;

        ZNavNode closestNode;
        vector3 closestPoint;
        double closestDistance = INT.Max;

        queue.push(node);
        addNodeToDepthList(node.nodeID, 0, nodeDepth);

        int runaway = 0;

        while (queue.size())
        {
            runaway++;
            if (runaway > 2000)
            {
                break;
            }

            ZNavNode currentNode = queue[queue.size() - 1];
            queue.delete(queue.size() - 1, 1);

            // Test closest point on triangle
            tri.set(
                currentNode.getVertexByIndex(0),
                currentNode.getVertexByIndex(1),
                currentNode.getVertexByIndex(2)
            );

            point = tri.closestPointToPoint(endPoint);
            double dist = Math.distanceToSquared(point, endPoint);

            if (dist < closestDistance)
            {
                closestNode = currentNode;
                closestPoint = point;
                closestDistance = dist;
            }

            int depth = getDepthFromList(currentNode.nodeID, nodeDepth);
            if (depth > 2) continue;

            // Expand to neighbors
            for (int i = 0; i < currentNode.neighborIDs.size(); i++)
            {
                ZNavNode neighbor = currentNode.getNeighborByIndex(i);
                if (depthListHasMember(neighbor.nodeID, nodeDepth))
                {
                    continue;
                }
                else
                {
                    addNodeToDepthList(neighbor.nodeID, depth + 1, nodeDepth);
                    queue.push(neighbor);
                }
            }
        }

        return closestPoint, closestNode;
    }

    /// <summary>
    /// Tests whether an agent has arrived near its destination.
    /// Uses bounding box intersection rather than exact distance.
    /// </summary>
    bool CheckArrival(ZNavAgent mo, vector3 dest)
    {
        bboxSrc.setFromActor(mo);
        bboxTarg.setFromCenterAndSize(dest + (0, 0, 64), (64, 64, 64));
        return ZNavAABB3.boxesIntersect(bboxSrc, bboxTarg);
    }
}

/// <summary>
/// Connected component of navigation nodes.
/// Agents cannot pathfind across groups.
/// </summary>
Class ZNavGroup : ZNav
{
    /// <summary>Group identifier.</summary>
    int groupID;

    /// <summary>Owning navigation mesh.</summary>
    ZNavMesh mesh;

    /// <summary>Nodes belonging to this group.</summary>
    array<ZNavNode> nodes;

    /// <summary>
    /// Returns closest node in this group to a position.
    /// </summary>
    ZNavNode getClosestNode(vector3 pos)
    {
        return mesh.getClosestNodeInGroup(pos, self.groupID);
    }

    /// <summary>
    /// Resets all nodes before a pathfinding search.
    /// </summary>
    void resetNodes()
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            self.nodes[i].clear();
        }
    }
}

/// <summary>
/// Navigation mesh polygon node.
/// Nodes form the graph used by A* pathfinding.
/// </summary>
Class ZNavNode : ZNav
{
    /* ---------- Network ---------- */

    /// <summary>Owning group.</summary>
    ZNavGroup group;

    /// <summary>Owning mesh.</summary>
    ZNavMesh mesh;

    /* ---------- Helper / Editor Data ---------- */

    /// <summary>Editor helper actor associated with this node.</summary>
    NavHelper helper;

    /// <summary>TID of helper actor.</summary>
    int helperTID;

    /* ---------- Node Properties ---------- */

    int flags;
    int nodeID;
    vector3 centroid;

    /* ---------- Geometry & Connectivity ---------- */

    int MaxNeighborIDs;
    array<int> neighborIDs;
    array<int> vertexIDs;
    array<ZNavPortal> portals;

    bool isLeap, isArc, isLanding;
    ZNavCell cell;

    /* ---------- A* Runtime State ---------- */

    ZNavNode parent;
    bool closed;
    bool visited;
    int cost;
    int f;
    int g;
    int h;

    /// <summary>
    /// Finds and links the NavHelper actor by TID.
    /// </summary>
    play void getHelper(int tid)
    {
        ThinkerIterator it = ThinkerIterator.Create('NavHelper');
        NavHelper nt;

        while (nt = NavHelper(it.Next()))
        {
            if (nt && nt.tid == tid)
            {
                helper = nt;
                break;
            }
        }

        if (!helper)
        {
            console.printf("\c[red]couldn't find helper node\cl");
        }
        else
        {
            helper.addNode(self);
        }
    }

    /// <summary>
    /// Returns closest point on polygon perimeter to a point.
    /// </summary>
    play vector3 pointOnPerimter(vector3 pt)
    {
        double closestDistance = Int.Max;
        vector2 closestPoint;

        int size = self.vertexIDs.size();

        for (int i = 0; i < size; i++)
        {
            int j = i + 1;
            if (j >= size) j = 0;

            vector3 a = getVertexByIndex(i);
            vector3 b = getVertexByIndex(j);

            vector2 c = Math.getClosestPointOnLine(a, b, pt);
            double dist = Math.distanceToSquared2(pt, (c, 0));

            if (dist < closestDistance)
            {
                closestDistance = dist;
                closestPoint = c;
            }
        }

        return (closestPoint, pt.z);
    }

    /// <summary>
    /// Ensures a point lies inside the polygon, clamping to edges if necessary.
    /// </summary>
    play vector3 constrainPoint(vector3 pt)
    {
        vector3 nextPt = pt;

        if (!ZNavMesh.isPointInNode(pt, self))
        {
            nextPt = self.pointOnPerimter(pt);
        }

        return nextPt;
    }

    /// <summary>
    /// Returns traversal cost of this node.
    /// </summary>
    int getCost()
    {
        return cost;
    }

    /// <summary>
    /// Returns vertex by global vertex ID.
    /// </summary>
    vector3 getVertex(int vertexID)
    {
        return self.mesh.getVertex(vertexID);
    }

    /// <summary>
    /// Returns vertex by local polygon index.
    /// </summary>
    vector3 getVertexByIndex(int index)
    {
        return self.mesh.getVertex(vertexIDs[index]);
    }

    /// <summary>
    /// Returns neighbor node by node ID.
    /// </summary>
    ZNavNode getNeighbor(int neighborID)
    {
        return self.mesh.nodes[neighborID];
    }

    /// <summary>
    /// Returns neighbor node by neighbor list index.
    /// </summary>
    ZNavNode getNeighborByIndex(int index)
    {
        return self.mesh.nodes[self.neighborIDs[index]];
    }

    /// <summary>
    /// Resets A* state variables before a new search.
    /// </summary>
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

    /// <summary>
    /// Returns nearest point on this polygon to the given position.
    /// </summary>
    vector3 getNearestPointTo(vector3 pos)
    {
        mesh.tri.set(getVertexByIndex(0), getVertexByIndex(1), getVertexByIndex(2));
        return mesh.tri.closestPointToPoint(pos);
    }
}




