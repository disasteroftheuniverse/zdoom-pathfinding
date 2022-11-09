/*
    Binary heap ported from
    https://github.com/donmccurdy/three-pathfinding/blob/main/src/BinaryHeap.js
*/
Class ZNavBinaryHeap : ZNav
{
    array<ZNavNode>content;

    virtual int scoreFunction(ZNavNode element) 
    {
        return element.f;
    }

    void push (ZNavNode element)
    {
        self.content.push(element);
        self.sinkDown( self.content.size() - 1 );
    }

    void clear()
    {
        self.content.clear();
    }

    ZNavNode pop()
    {
        ZNavNode result = self.content[0];
        ZNavNode last = self.content[self.content.size() - 1];

        self.content.pop();

        if ( self.content.size() > 0 )
        {
            self.content[0] = last;
            bubbleUp(0);
        }

        return result;
    }

    void remove ( ZNavNode element  )
    {
        int index = content.find(element);
        ZNavNode last = self.content[self.content.size() - 1];
        self.content.pop();

        if ( index == (content.size() - 1) )
        {
            content[index] = last;

            if ( scoreFunction( last ) < scoreFunction( element ) )
            {
                sinkDown(index);
            } else {
                bubbleUp(index);
            }
        }
    }

    void rescoreElement ( ZNavNode element )
    {
        int index = self.content.find(element);
        sinkDown(index);
    }

    int size()
    {
        return self.content.size();
    }

    void sinkDown ( int n )
    {
        if ( n >= self.content.size() ) return;
        ZNavNode element = content[n];

        while ( n > 0 )
        {
            int parentN = ((n + 1) >> 1) - 1;
            if ( parentN >= content.size() ) break;

            ZNavNode parent = content[parentN];

            if (scoreFunction (element) < scoreFunction (parent) )
            {
                content[parentN] = element;
                content[n] = parent;
            } else {
                break;
            }
        }
    }

    void bubbleUp( int n )
    {
        int length = self.content.size();
        if (n >= length ) return;

        ZNavNode element = self.content[n];
        int elemScore = scorefunction(element);

        while (true)
        {
            int child2N = (n + 1) << 1;
            int child1N = child2N - 1;

            int swap = 0;
            bool didSwap = false;
            int child1Score;

            if (child1N < length)
            {
                ZNavNode child1 = content[child1N];
                child1Score = scorefunction(child1);

                if (child1Score < elemScore)
                {
                    swap = child1N;
                    didSwap = true;
                }
            }

            if (child2N < length)
            {
                ZNavNode child2 = content[child2N];
                int child2Score = scorefunction(child2);
                int swapped = (didSwap == false) ? elemScore : child1Score;

                if (child2Score < swapped )
                {
                    swap = child2N;
                    didSwap = true;
                }

            }

            if ( didSwap == true )
            {
                content[n] = content[swap];
                content[swap] = element;
                n = swap;

            } else {
                break;
            }
        }
    }
}

/*
    AStar ported from 
    https://github.com/donmccurdy/three-pathfinding/blob/main/src/AStar.js
*/
Class ZNavAStar : ZNav
{
    static bool Search (
        ZNavGroup group, 
        ZNavNode startNode, 
        ZNavNode endNode, 
        in out array<ZNavNode> nodes 
    )
    {
        group.resetNodes();
        ZNavBinaryHeap openHeap = new ('ZNavBinaryHeap');
        openHeap.push(startNode);
        while ( openHeap.size() )
        {
            ZNavNode currentNode = openHeap.pop();

            if (currentNode == endNode)
            {
                ZNavNode curr = currentNode;

                array<ZNavNode>ret;
                while (curr.parent)
                {
                    ret.push(curr);
                    curr = curr.parent;
                }

                array<ZNavNode>rev;
                for (int j = ret.size() - 1; j > -1; j--)
                {
                    curr = ret[j];
                    rev.push(curr);
                }

                ret.clear();
                nodes.move( rev );
                openHeap.destroy();
                return true;
            }

            currentNode.closed = true;
            for (int i = 0; i < currentNode.neighborIDs.size(); i++ )
            {
                int neighborID = currentNode.neighborIDs[i];
                ZNavNode neighbor = currentNode.getNeighbor( neighborID );

                if (neighborID == currentNode.nodeID) 
                {
                    continue;
                }

                if (neighbor.closed) 
                {
                    continue;
                }

                int gScore = currentNode.g + neighbor.getCost();
                // currentNode.neighborCosts[i];
                bool beenVisited = neighbor.visited;

                if (!beenVisited || gScore < neighbor.g)
                {
                    neighbor.visited = true;
                    neighbor.parent = currentNode;

                    neighbor.h = ( neighbor.h == 0 ) ? Math.CheapDistance( neighbor.centroid, endNode.centroid ) : neighbor.h;
                    neighbor.g = gScore;
                    neighbor.f = neighbor.g + neighbor.h;

                    if (!beenVisited)
                    {
                        openHeap.push(neighbor);
                    } else {
                        openHeap.rescoreElement(neighbor);
                    }
                }
            }
        }
        openHeap.destroy();
        return false;
    }
}

class ZNavRoute : ZNav
{
    array<double>navpoints;
    //navhelper goal;

    bool hasRoute()
    {
        return self.navpoints.size() > 0;
    }

    static ZNavRoute create ()
    {
        ZNavRoute route = new ('ZNavRoute');
        return route;
    }

    ZNavRoute clone()
    {
        ZNavRoute route = ZNavRoute.create();
        route.copy(self);
        return route;
    }

    void copy( ZNavRoute route )
    {
        self.navpoints.copy( route.navpoints );
        //self.goal = route.goal;
    }

    void dispose()
    {
        self.clear();
        self.destroy();
    }

    void clear()
    {
        //goal = null;
        navpoints.clear();
    }

    bool has ( vector3 v )
    {
        return indexOf (v) > -1;
    }

    int indexOf (vector3 v)
    {
        for (int i = 0; i < self.size(); i++)
        {
            if ( v == self.get(i) )
            {
                return i;
            }
        }
        return -1;
    }

    int size ()
    {
        return navpoints.size() / 3;
    }

    void delete( int index, int howmany = 1)
    {
        index*=3;
        navpoints.delete(index, 3 * howmany);
    }

    void push ( vector3 v )
    {
        navpoints.push(v.x);
        navpoints.push(v.y);
        navpoints.push(v.z);
    }

    vector3 pop()
    {
        int size = self.size();
        vector3 ret = self.get( size - 1 );
        self.delete(size - 1);
        return ret;
    }

    vector3 shift()
    {
        vector3 ret = self.get( 0 );
        self.delete(0);
        return ret;
    }

    bool shift2()
    {
        if ( self.size() )
        {
            self.delete(0);
            return true;
        }
        return false;
    }

    vector3 get( int index )
    {
        index*=3;
        return (navpoints[index], navpoints[index+1], navpoints[index+2]);
    }

    void fromPoints ( in array<double> points )
    {
        self.navpoints.move ( points );
    }

    /*void addGoal (navhelper goal)
    {
        self.goal = goal;
    }*/
}