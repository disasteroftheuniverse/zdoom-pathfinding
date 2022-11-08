Class ZNavPortal : ZNav
{
    array<Int>vertexIDs;
    ZNavMesh mesh;
    
    vector3 getVertex (int vertexID)
    {
        return self.mesh.getVertex( vertexID );
    }
}

class ZNavChannel : ZNav
{
    int length;
    array<double>positions;

    static ZNavChannel Create()
    {
        ZNavChannel channel = new ('ZNavChannel');
        return channel;
    }

    static void addPositionTo (vector3 pos, in out array<double> positions)
    {
        positions.push(pos.x);
        positions.push(pos.y);
        positions.push(pos.z);
    }

    static vector3 getSingle ( int index, in array<double> positions )
    {
        index *= 3;
        return ( positions[index], positions[index+1], positions[index+2] );
    }

    static bool _vequal(vector3 a, vector3 b)
    {
        return Math.distanceToSquared2(a,b) <= 40.0;
    }

    static int getSizeSingle(in array<double> positions)
    {
        return positions.size() / 3;
    }

    static int getSizePairs(in array<double> positions)
    {
        return positions.size() / 6;
    }

    void dispose()
    {
        self.clear();
        self.destroy();
    }

    void clear()
    {
        self.positions.clear();
        self.length = 0;
    }

    bool vequal(vector3 a, vector3 b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    double triarea2(vector3 a, vector3 b, vector3 c)
    {
        return Math.triarea2( a, b, c );
    }

    void addPair (vector3 vl, vector3 vr )
    {
        ZNavChannel.addPositionTo( vl , self.positions );
        ZNavChannel.addPositionTo( vr , self.positions );
        self.length+=1;
    }

    void addSingle ( vector3 v )
    {
        ZNavChannel.addPositionTo( v, self.positions );
        ZNavChannel.addPositionTo( v, self.positions );
        self.length+=1;
    }

    vector3 getLeft( int index )
    {
        index *= 6;
        return ( self.positions[index], self.positions[index+1], self.positions[index+2] );
    }

    vector3 getRight ( int index )
    {
        index *= 6;
        return ( self.positions[index+3], self.positions[index+4], self.positions[index+5] );
    }

    Vector3, Vector3 getPair ( int index )
    {
        index = index * 6;
        return self.getLeft( index ), self.getRight( index );
    }

    static vector3 getLastItem ( in out array<double>points )
    {
        int lastIndex = (points.size() / 3) - 1;
        return ZNavChannel.getSingle( lastIndex, points);
    }

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

            //right
            if (  triarea2( portalApex, portalRight, right ) >= 0.0 )
            {
                if ( vequal( portalApex, portalRight ) ||  triarea2( portalApex, portalLeft, right ) < 0.0 )
                {
                    portalRight = right;
                    rightIndex = i;

                } else {
                    
                    //pts.push(portalLeft);
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

            //left
            if ( triarea2( portalApex, portalLeft, left ) <= 0.0 )
            {
                if ( vequal( portalApex, portalLeft ) || triarea2( portalApex, portalRight, left ) > 0.0 )
                {
                    portalLeft = left;
                    leftIndex = i;
                } else {

                    //pts.push(portalRight);
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