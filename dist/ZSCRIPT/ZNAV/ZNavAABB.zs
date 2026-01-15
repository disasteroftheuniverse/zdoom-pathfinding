/// <summary>
/// Simple axis-aligned bounding box (AABB) in 3D space.
/// Used for fast intersection and containment tests.
/// </summary>
/// <remarks>
/// Typical uses:
/// - Build from an Actor using <see cref="setFromActor"/>
/// - Expand to include points using <see cref="expandByVector"/>
/// - Test overlap using <see cref="intersectsAABB3"/>
/// </remarks>
struct ZNavAABB3
{
    /// <summary>Minimum corner (lowest X, Y, Z) of the box.</summary>
    vector3 minExt;

    /// <summary>Maximum corner (highest X, Y, Z) of the box.</summary>
    vector3 maxExt;

    /// <summary>
    /// Tests whether two AABBs intersect or touch.
    /// Uses separating axis checks along X, Y, and Z.
    /// </summary>
    /// <param name="box1">First bounding box.</param>
    /// <param name="box2">Second bounding box.</param>
    /// <returns>True if boxes overlap or touch; otherwise false.</returns>
    static bool boxesIntersect(ZNavAABB3 box1, ZNavAABB3 box2)
    {
        // Using 6 separating planes (min/max on each axis) to rule out intersections.
        // If any axis is separated, boxes do not intersect.
        return box1.maxExt.x < box2.minExt.x || box1.minExt.x > box2.maxExt.x ||
               box1.maxExt.y < box2.minExt.y || box1.minExt.y > box2.maxExt.y ||
               box1.maxExt.z < box2.minExt.z || box1.minExt.z > box2.maxExt.z
            ? false
            : true;
    }

    /// <summary>
    /// Returns the smaller of two values.
    /// </summary>
    double min(double a, double b)
    {
        return a < b ? a : b;
    }

    /// <summary>
    /// Returns the larger of two values.
    /// </summary>
    double max(double a, double b)
    {
        return a > b ? a : b;
    }

    /// <summary>
    /// Resets the box to an "infinite" size, effectively containing everything.
    /// Useful before incrementally expanding with points.
    /// </summary>
    void clear()
    {
        self.minExt = (-INT.Max, -INT.Max, -INT.Max);
        self.maxExt = ( INT.Max,  INT.Max,  INT.Max);
    }

    /// <summary>
    /// Sets the bounding box to match an actor's cylindrical bounds.
    /// Uses actor radius for X/Y and height for Z.
    /// </summary>
    /// <param name="mo">Actor to build bounds from.</param>
    void setFromActor(actor mo)
    {
        if (!mo) return;

        // Horizontal extents from radius, vertical extents from height
        self.minExt = (mo.pos.x - mo.radius, mo.pos.y - mo.radius, mo.pos.z);
        self.maxExt = (mo.pos.x + mo.radius, mo.pos.y + mo.radius, mo.pos.z + mo.height);
    }

    /// <summary>
    /// Expands this box so that it also contains the given point.
    /// </summary>
    /// <param name="v">Point to include in the bounds.</param>
    void expandByVector(vector3 v)
    {
        self.minExt = (min(self.minExt.x, v.x),
                       min(self.minExt.y, v.y),
                       min(self.minExt.z, v.z));

        self.maxExt = (max(self.maxExt.x, v.x),
                       max(self.maxExt.y, v.y),
                       max(self.maxExt.z, v.z));
    }

    /// <summary>
    /// Tests whether a point is within the box on the Z axis only.
    /// </summary>
    /// <param name="pt">Point to test.</param>
    /// <returns>True if Z is within bounds; otherwise false.</returns>
    bool containsPointZ(vector3 pt)
    {
        return pt.z < self.minExt.z || pt.z > self.maxExt.z ? false : true;
    }

    /// <summary>
    /// Tests whether a point lies fully inside the box.
    /// </summary>
    /// <param name="pt">Point to test.</param>
    /// <returns>True if point is inside all extents; otherwise false.</returns>
    bool containsPoint(vector3 pt)
    {
        return pt.x < self.minExt.x || pt.x > self.maxExt.x ||
               pt.y < self.minExt.y || pt.y > self.maxExt.y ||
               pt.z < self.minExt.z || pt.z > self.maxExt.z
            ? false
            : true;
    }

    /// <summary>
    /// Tests whether the given box is fully contained within this box.
    /// </summary>
    /// <param name="box">Box to test for containment.</param>
    /// <returns>True if the entire box fits inside this box.</returns>
    bool containsBox(ZNavAABB3 box)
    {
        return self.minExt.x <= box.minExt.x && box.maxExt.x <= self.maxExt.x &&
               self.minExt.y <= box.minExt.y && box.maxExt.y <= self.maxExt.y &&
               self.minExt.z <= box.minExt.z && box.maxExt.z <= self.maxExt.z;
    }

    /// <summary>
    /// Tests whether this box intersects another box.
    /// </summary>
    /// <param name="box">Other box to test.</param>
    /// <returns>True if boxes overlap or touch.</returns>
    bool intersectsAABB3(ZNavAABB3 box)
    {
        return ZNavAABB3.boxesIntersect(self, box);
    }

    /// <summary>
    /// Returns the center point of the bounding box.
    /// </summary>
    vector3 getCenter()
    {
        return (self.minExt + self.maxExt) * 0.5;
    }

    /// <summary>
    /// Returns the full size (width, depth, height) of the box.
    /// </summary>
    vector3 getSize()
    {
        return self.maxExt - self.minExt;
    }

    /// <summary>
    /// Sets the bounding box using a center point and full size.
    /// </summary>
    /// <param name="center">Center of the box.</param>
    /// <param name="size">Full size of the box on each axis.</param>
    void setFromCenterAndSize(vector3 center, vector3 size)
    {
        self.minExt = (center.x - size.x / 2,
                       center.y - size.y / 2,
                       center.z - size.z / 2);

        self.maxExt = (center.x + size.x / 2,
                       center.y + size.y / 2,
                       center.z + size.z / 2);
    }
}

/// <summary>
/// Mixin that adds a reusable 3D axis-aligned bounding box to a class.
/// Useful for actors or components that frequently perform spatial tests.
/// </summary>
mixin class boundingbox
{
    /// <summary>Bounding box associated with the owning object.</summary>
    ZNavAABB3 bbox;
}
