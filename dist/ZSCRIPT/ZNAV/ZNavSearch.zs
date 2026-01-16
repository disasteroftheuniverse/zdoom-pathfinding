/// <summary>
/// Binary min-heap implementation used for fast retrieval of the lowest-cost node.
/// Ported from three-pathfinding's BinaryHeap.js implementation.
/// </summary>
/// <remarks>
/// This is a utility class used by A* search to efficiently manage the open set.
/// You should not need to modify this unless changing core pathfinding behavior.
/// </remarks>
Class ZNavBinaryHeap : ZNav
{
    /// <summary>Internal array storing heap elements.</summary>
    array<ZNavNode> content;

    /// <summary>
    /// Returns the score used to order elements in the heap.
    /// Lower scores have higher priority.
    /// </summary>
    /// <param name="element">Node to score.</param>
    /// <returns>Score value (usually f = g + h).</returns>
    virtual int scoreFunction(ZNavNode element)
    {
        return element.f;
    }

    /// <summary>
    /// Inserts a node into the heap and restores heap order.
    /// </summary>
    /// <param name="element">Node to add.</param>
    void push(ZNavNode element)
    {
        self.content.push(element);
        // Move the element upward until heap property is restored
        self.sinkDown(self.content.size() - 1);
    }

    /// <summary>
    /// Removes all elements from the heap.
    /// </summary>
    void clear()
    {
        self.content.clear();
    }

    /// <summary>
    /// Removes and returns the node with the lowest score.
    /// </summary>
    /// <returns>Node with minimum score.</returns>
    ZNavNode pop()
    {
        // Root of heap is always the minimum element
        ZNavNode result = self.content[0];
        ZNavNode last = self.content[self.content.size() - 1];

        self.content.pop();

        // Move last element to root and restore heap property
        if (self.content.size() > 0)
        {
            self.content[0] = last;
            bubbleUp(0);
        }

        return result;
    }

    /// <summary>
    /// Removes a specific node from the heap.
    /// </summary>
    /// <param name="element">Node to remove.</param>
    void remove(ZNavNode element)
    {
        int index = content.find(element);
        ZNavNode last = self.content[self.content.size() - 1];
        self.content.pop();

        // If not removing the last element, restore heap ordering
        if (index == (content.size() - 1))
        {
            content[index] = last;

            if (scoreFunction(last) < scoreFunction(element))
            {
                sinkDown(index);
            }
            else
            {
                bubbleUp(index);
            }
        }
    }

    /// <summary>
    /// Reorders the heap after a node's score has changed.
    /// </summary>
    /// <param name="element">Node whose score was updated.</param>
    void rescoreElement(ZNavNode element)
    {
        int index = self.content.find(element);
        sinkDown(index);
    }

    /// <summary>
    /// Returns the number of elements currently in the heap.
    /// </summary>
    int size()
    {
        return self.content.size();
    }

    /// <summary>
    /// Moves a node up the heap until heap ordering is restored.
    /// </summary>
    /// <param name="n">Index of the element to sink.</param>
    void sinkDown(int n)
    {
        if (n >= self.content.size()) return;

        ZNavNode element = content[n];

        while (n > 0)
        {
            // Parent index in binary heap
            int parentN = ((n + 1) >> 1) - 1;
            if (parentN >= content.size()) break;

            ZNavNode parent = content[parentN];

            // Swap if element has lower score than parent
            if (scoreFunction(element) < scoreFunction(parent))
            {
                content[parentN] = element;
                content[n] = parent;
                n = parentN;
            }
            else
            {
                break;
            }
        }
    }

    /// <summary>
    /// Moves a node down the heap until heap ordering is restored.
    /// </summary>
    /// <param name="n">Index of the element to bubble.</param>
    void bubbleUp(int n)
    {
        int length = self.content.size();
        if (n >= length) return;

        ZNavNode element = self.content[n];
        int elemScore = scoreFunction(element);

        while (true)
        {
            // Child indices
            int child2N = (n + 1) << 1;
            int child1N = child2N - 1;

            int swap = 0;
            bool didSwap = false;
            int child1Score;

            // Check left child
            if (child1N < length)
            {
                ZNavNode child1 = content[child1N];
                child1Score = scoreFunction(child1);

                if (child1Score < elemScore)
                {
                    swap = child1N;
                    didSwap = true;
                }
            }

            // Check right child
            if (child2N < length)
            {
                ZNavNode child2 = content[child2N];
                int child2Score = scoreFunction(child2);
                int swappedScore = (didSwap == false) ? elemScore : child1Score;

                if (child2Score < swappedScore)
                {
                    swap = child2N;
                    didSwap = true;
                }
            }

            // Perform swap if needed
            if (didSwap == true)
            {
                content[n] = content[swap];
                content[swap] = element;
                n = swap;
            }
            else
            {
                break;
            }
        }
    }
}

/// <summary>
/// A* pathfinding implementation over navigation mesh nodes.
/// Ported from three-pathfinding's AStar.js.
/// </summary>
/// <remarks>
/// Searches through navigation nodes to find the shortest path between two nodes.
/// Returns an ordered list of nodes representing the path.
/// </remarks>
Class ZNavAStar : ZNav
{
    /// <summary>
    /// Performs A* search on a navigation group.
    /// </summary>
    /// <param name="group">Navigation group containing all nodes.</param>
    /// <param name="startNode">Node nearest to the starting position.</param>
    /// <param name="endNode">Node nearest to the target position.</param>
    /// <param name="nodes">Output list of nodes forming the final path.</param>
    /// <returns>True if a path was found; otherwise false.</returns>
    static bool Search(
        ZNavGroup group,
        ZNavNode startNode,
        ZNavNode endNode,
        in out array<ZNavNode> nodes
    )
    {
        // Reset all node state before searching
        group.resetNodes();

        // Open set implemented as binary heap (min by f-score)
        ZNavBinaryHeap openHeap = new('ZNavBinaryHeap');
        openHeap.push(startNode);

        while (openHeap.size())
        {
            // Node with lowest estimated total cost
            ZNavNode currentNode = openHeap.pop();

            // Path found — reconstruct path by walking parents
            if (currentNode == endNode)
            {
                ZNavNode curr = currentNode;

                array<ZNavNode> ret;
                while (curr.parent)
                {
                    ret.push(curr);
                    curr = curr.parent;
                }

                // Reverse to get start → end order
                array<ZNavNode> rev;
                for (int j = ret.size() - 1; j > -1; j--)
                {
                    curr = ret[j];
                    rev.push(curr);
                }

                ret.clear();
                nodes.move(rev);
                openHeap.destroy();
                return true;
            }

            currentNode.closed = true;

            // Evaluate all neighbors
            for (int i = 0; i < currentNode.neighborIDs.size(); i++)
            {
                int neighborID = currentNode.neighborIDs[i];
                ZNavNode neighbor = currentNode.getNeighbor(neighborID);

                // Skip self-links
                if (neighborID == currentNode.nodeID)
                {
                    continue;
                }

                // Skip already-processed nodes
                if (neighbor.closed)
                {
                    continue;
                }

                // Cost from start through current node
                int gScore = currentNode.g + neighbor.getCost();
                bool beenVisited = neighbor.visited;

                // If this path is better, update neighbor
                if (!beenVisited || gScore < neighbor.g)
                {
                    neighbor.visited = true;
                    neighbor.parent = currentNode;

                    // Heuristic estimated distance to goal (cached)
                    neighbor.h = (neighbor.h == 0)
                        ? Math.CheapDistance(neighbor.centroid, endNode.centroid)
                        : neighbor.h;

                    neighbor.g = gScore;
                    neighbor.f = neighbor.g + neighbor.h;

                    if (!beenVisited)
                    {
                        openHeap.push(neighbor);
                    }
                    else
                    {
                        openHeap.rescoreElement(neighbor);
                    }
                }
            }
        }

        openHeap.destroy();
        return false;
    }
}

/// <summary>
/// Represents a path as an ordered list of 3D points.
/// Agents can follow these points sequentially.
/// </summary>
/// <remarks>
/// Internally stores points as a flat array of doubles: [x, y, z, x, y, z, ...]
/// </remarks>
class ZNavRoute : ZNav
{
    /// <summary>Flat array storing XYZ triplets for each point.</summary>
    array<double> navpoints;

    /// <summary>
    /// Returns true if the route contains any points.
    /// </summary>
    bool hasRoute()
    {
        return self.navpoints.size() > 0;
    }

    /// <summary>
    /// Creates a new empty route.
    /// </summary>
    static ZNavRoute create()
    {
        ZNavRoute route = new('ZNavRoute');
        return route;
    }

    /// <summary>
    /// Creates a deep copy of this route.
    /// </summary>
    ZNavRoute clone()
    {
        ZNavRoute route = ZNavRoute.create();
        route.copy(self);
        return route;
    }

    /// <summary>
    /// Copies all points from another route into this one.
    /// </summary>
    /// <param name="route">Route to copy from.</param>
    void copy(ZNavRoute route)
    {
        self.navpoints.copy(route.navpoints);
    }

    /// <summary>
    /// Clears all points and destroys this route object.
    /// </summary>
    void dispose()
    {
        self.clear();
        self.destroy();
    }

    /// <summary>
    /// Removes all points from the route.
    /// </summary>
    void clear()
    {
        navpoints.clear();
    }

    /// <summary>
    /// Returns true if the route contains the specified point.
    /// </summary>
    bool has(vector3 v)
    {
        return indexOf(v) > -1;
    }

    /// <summary>
    /// Returns the index of the given point in the route.
    /// </summary>
    /// <returns>Point index, or -1 if not found.</returns>
    int indexOf(vector3 v)
    {
        for (int i = 0; i < self.size(); i++)
        {
            if (v == self.get(i))
            {
                return i;
            }
        }
        return -1;
    }

    /// <summary>
    /// Returns the number of points in the route.
    /// </summary>
    int size()
    {
        return navpoints.size() / 3;
    }

    /// <summary>
    /// Deletes one or more points starting at the given index.
    /// </summary>
    /// <param name="index">Point index.</param>
    /// <param name="howmany">Number of points to remove.</param>
    void delete(int index, int howmany = 1)
    {
        index *= 3;
        navpoints.delete(index, 3 * howmany);
    }

    /// <summary>
    /// Appends a point to the end of the route.
    /// </summary>
    void push(vector3 v)
    {
        navpoints.push(v.x);
        navpoints.push(v.y);
        navpoints.push(v.z);
    }

    /// <summary>
    /// Removes and returns the last point in the route.
    /// </summary>
    vector3 pop()
    {
        int size = self.size();
        vector3 ret = self.get(size - 1);
        self.delete(size - 1);
        return ret;
    }

    /// <summary>
    /// Removes and returns the first point in the route.
    /// </summary>
    vector3 shift()
    {
        vector3 ret = self.get(0);
        self.delete(0);
        return ret;
    }

    /// <summary>
    /// Removes the first point without returning it.
    /// </summary>
    /// <returns>True if a point was removed.</returns>
    bool shift2()
    {
        if (self.size())
        {
            self.delete(0);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Returns the point at the specified index.
    /// </summary>
    vector3 get(int index)
    {
        index *= 3;
        return (navpoints[index], navpoints[index + 1], navpoints[index + 2]);
    }

    /// <summary>
    /// Replaces route points using a flat XYZ array.
    /// </summary>
    /// <param name="points">Flat array of XYZ values.</param>
    void fromPoints(in array<double> points)
    {
        self.navpoints.move(points);
    }
}
