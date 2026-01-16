/* 
    A portal just describes a line which is shared between two
    nodes. Edges shared by a polygon, basically.

    The whole purpose of ZNavPortal and ZNavChannel
    is to take a list of nodes, and make a smooth path 
    between the start node and end node.

    This just smooths out the path.
    There should be no reason to modify this at all.
*/

/// <summary>
/// Represents a portal connecting two nodes in a navigation mesh.
/// </summary>
Class ZNavPortal : ZNav
{
    /// <summary>
    /// IDs of vertices that define the portal.
    /// </summary>
    array<Int>vertexIDs;

    /// <summary>
    /// Reference to the navigation mesh containing this portal.
    /// </summary>
    ZNavMesh mesh;
    
    /// <summary>
    /// Returns the 3D position of a vertex given its ID.
    /// </summary>
    /// <param name="vertexID">The ID of the vertex to retrieve.</param>
    /// <returns>3D position of the vertex as a vector3.</returns>
    vector3 getVertex (int vertexID)
    {
        return self.mesh.getVertex( vertexID );
    }
}

/// <summary>
/// Represents a navigation channel, used to smooth a path through portals.
/// Contains an array of positions for the left and right edges of the channel.
/// </summary>
class ZNavChannel : ZNav
{
    /// <summary>
    /// Number of pairs of positions in the channel.
    /// </summary>
    int length;

    /// <summary>
    /// Flattened array of 3D positions (x, y, z) representing channel edges.
    /// </summary>
    array<double>positions;

    /// <summary>
    /// Creates a new instance of ZNavChannel.
    /// </summary>
    /// <returns>A new ZNavChannel instance.</returns>
    static ZNavChannel Create()
    {
        ZNavChannel channel = new ('ZNavChannel');
        return channel;
    }

    /// <summary>
    /// Adds a 3D position to the given positions array.
    /// </summary>
    /// <param name="pos">The 3D position to add.</param>
    /// <param name="positions">Reference to the positions array.</param>
    static void addPositionTo (vector3 pos, in out array<double> positions)
    {
        positions.push(pos.x);
        positions.push(pos.y);
        positions.push(pos.z);
    }

    /// <summary>
    /// Retrieves a single 3D position from the flattened array.
    /// </summary>
    /// <param name="index">Index of the position.</param>
    /// <param name="positions">Array containing positions.</param>
    /// <returns>The 3D position as a vector3.</returns>
    static vector3 getSingle ( int index, in array<double> positions )
    {
        index *= 3;
        return ( positions[index], positions[index+1], positions[index+2] );
    }

    /// <summary>
    /// Checks if two vectors are approximately equal using a threshold distance.
    /// </summary>
    /// <param name="a">First vector.</param>
    /// <param name="b">Second vector.</param>
    /// <returns>True if vectors are within a small distance; otherwise false.</returns>
    static bool _vequal(vector3 a, vector3 b)
    {
        return Math.distanceToSquared2(a,b) <= 40.0;
    }

    /// <summary>
    /// Returns the number of single positions in the flattened array.
    /// </summary>
    static int getSizeSingle(in array<double> positions)
    {
        return positions.size() / 3;
    }

    /// <summary>
    /// Returns the number of position pairs in the flattened array.
    /// </summary>
    static int getSizePairs(in array<double> positions)
    {
        return positions.size() / 6;
    }

    /// <summary>
    /// Clears the channel and disposes of resources.
    /// </summary>
    void dispose()
    {
        self.clear();
        self.destroy();
    }

    /// <summary>
    /// Clears the positions and resets the length.
    /// </summary>
    void clear()
    {
        self.positions.clear();
        self.length = 0;
    }

    /// <summary>
    /// Checks if two vectors are exactly equal.
    /// </summary>
    bool vequal(vector3 a, vector3 b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    /// <summary>
    /// Computes the signed area of a triangle defined by three points.
    /// </summary>
    double triarea2(vector3 a, vector3 b, vector3 c)
    {
        return Math.triarea2( a, b, c );
    }

    /// <summary>
    /// Adds a pair of vectors (left and right) to the channel positions.
    /// </summary>
    void addPair (vector3 vl, vector3 vr )
    {
        ZNavChannel.addPositionTo( vl , self.positions );
        ZNavChannel.addPositionTo( vr , self.positions );
        self.length+=1;
    }

    /// <summary>
    /// Adds a single vector as both left and right positions in the channel.
    /// </summary>
    void addSingle ( vector3 v )
    {
        ZNavChannel.addPositionTo( v, self.positions );
        ZNavChannel.addPositionTo( v, self.positions );
        self.length+=1;
    }

    /// <summary>
    /// Retrieves the left position for the given index.
    /// </summary>
    vector3 getLeft( int index )
    {
        index *= 6;
        return ( self.positions[index], self.positions[index+1], self.positions[index+2] );
    }

    /// <summary>
    /// Retrieves the right position for the given index.
    /// </summary>
    vector3 getRight ( int index )
    {
        index *= 6;
        return ( self.positions[index+3], self.positions[index+4], self.positions[index+5] );
    }

    /// <summary>
    /// Returns both left and right positions as a pair.
    /// </summary>
    Vector3, Vector3 getPair ( int index )
    {
        index = index * 6;
        return self.getLeft( index ), self.getRight( index );
    }

    /// <summary>
    /// Retrieves the last single position in a positions array.
    /// </summary>
    static vector3 getLastItem ( in out array<double>points )
    {
        int lastIndex = (points.size() / 3) - 1;
        return ZNavChannel.getSingle( lastIndex, points);
    }

    /// <summary>
    /// Applies the string-pulling algorithm to smooth the path through the channel.
    /// Optionally outputs the smoothed path to a ZNavRoute.
    /// </summary>
    /// <param name="route">Optional route object to store the smoothed path.</param>
    void stringPull( ZNavRoute route = null )
    {
        array<double>pts;
        vector3 portalApex, portalLeft, portalRight;

        int apexIndex = 0;
        int leftIndex = 0;
        int rightIndex = 0;

        portalApex = self.getLeft( 0 );
        portalLeft = self.getLeft( 0 );
        portalRight = self.getRight( 0 );

        ZNavChannel.addPositionTo( portalApex, pts);

        for (int i = 1; i < self.length; i++)
        {
            vector3 left = self.getLeft(i);
            vector3 right = self.getRight(i);

            // Update right portal
            if (  triarea2( portalApex, portalRight, right ) >= 0.0 )
            {
                if ( vequal( portalApex, portalRight ) ||  triarea2( portalApex, portalLeft, right ) < 0.0 )
                {
                    portalRight = right;
                    rightIndex = i;

                } else {
                    
                    ZNavChannel.addPositionTo( portalLeft, pts);
                    portalApex = portalLeft;
                    apexIndex = leftIndex;
                    // Reset portal
                    portalLeft = portalApex;
                    portalRight = portalApex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    // Restart scan
                    i = apexIndex;
                    continue;
                }
            }

            // Update left portal
            if ( triarea2( portalApex, portalLeft, left ) <= 0.0 )
            {
                if ( vequal( portalApex, portalLeft ) || triarea2( portalApex, portalRight, left ) > 0.0 )
                {
                    portalLeft = left;
                    leftIndex = i;
                } else {

                    ZNavChannel.addPositionTo( portalRight, pts);
                    portalApex = portalRight;
                    apexIndex = rightIndex;
                    // Reset portal
                    portalLeft = portalApex;
                    portalRight = portalApex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    // Restart scan
                    i = apexIndex;
                    continue;
                }
            }
        }

        // Ensure the last left point is included
        int lastSingleIndexPTS = ZNavChannel.getSizeSingle( pts ) - 1;
        int lastLeftIndex = ZNavChannel.getSizePairs( self.positions ) - 1;
        vector3 lastpoint = ZNavChannel.getSingle (lastSingleIndexPTS, pts );
        vector3 lastleft = getLeft (lastLeftIndex );
        bool lastisLeft = vequal ( lastpoint,  lastleft);

        if ( 
            ( pts.size() == 0) || 
            ( !lastisLeft ) 
        )
        {
            ZNavChannel.addPositionTo ( lastleft, pts );
        }

        if (route)
        {
            route.fromPoints ( pts );
        }
    }
}
