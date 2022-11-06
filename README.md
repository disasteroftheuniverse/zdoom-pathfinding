# ZDoom Pathfinding

Navmesh Pathfinding for ZDoom Actors.

## Prerequisites

Generate nav mesh JSON with **[zdoom-navmesh-generator](https://github.com/disasteroftheuniverse/zdoom-navmesh-generator)**

The parser requires **[ZJSON](https://github.com/RicardoLuis0/ZJSON)**

You will need to spawn an instance of `ZNavThinker` at load time in your maps.

## PK3 Structure

<pre> 
<b>Root/</b>
├── <b>ZJSON/</b> #place your navigation meshes in here
└── <b>ZSCRIPT/</b>
    ├── <b>ZNAV/</b> #place ZNAV contents here
    └── <b>ZJSON/</b> #place <a href="https://github.com/RicardoLuis0/ZJSON">ZJSON</a>  here
</pre>

## API

### ZNavNode

A node is a polygon area defined by a set of vertices, portals and a centroid.

##### Properties

- `groupID` **Int** Index of **ZNavGroup** to which this node belongs
- `Centroid` **vector3** The mean position of all vertices in the node.
- `vertexIDs` **array&lt;Int>** List of indices of vertices used to define the polygon.
- `neighborIDs` **array&lt;Int>** List of indices of **ZNavNodes** sharing edges with this polygon.
- `portals` **array&lt;ZNavPortal>** List of **ZNavPortals**, pairs of vertexIDs which define edges shared by neighboring nodes.


### ZNavGroup

A group is a collection of interconnected nodes. It is always possible to move between two nodes so long as they are in the same group.

### ZNavMesh

Class containing navigation mesh data from ZJSON.

#### getVertex

##### Parameters

-   `vertexID` **Int** Index of vertex.

Returns **Vector3** Position of Mesh vertex at index.

#### FindPath

##### Parameters

-   `groupID` **Int** Group of nodes to search for paths.
-   `startPos` **Vector3** Start Positions.
-   `endPos` **Vector3** Destination.
-   `route` **ZNavRoute** Reference to instance of ZNavRoute, this countains a list of points which define a path.

Returns **boolean** Whether or not it could find a path to destination.

### ZNavAgent

Base class for pathfinding actors.

#### A_MoveToEx

Move towards actor goal ( the actor's target, by default ).

### ZNavRoute

An object which contains paths composed of points (**Vector3**).

#### get

##### Parameters

-   `index` **Int** Get a vertex from list

Returns **Vector3** Point from a list of points.

#### shift

Returns **Vector3** Returns the first point in the route and removes it from the route.

#### pop

Returns **Vector3** Returns the last point in the route and removes it from the route.

#### push

##### Parameters

-   `point` **Vector3** Adds a point to the route.

#### clear

Remove all points from route


