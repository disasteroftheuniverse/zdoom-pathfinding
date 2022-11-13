// just a basic axis aligned bounding box
// use this to quickly check if objects are touching each other

/*
    setFromActor() - set bounding box from actor
    intersectsAABB3() - is this box touching another box?
*/

struct ZNavAABB3
{
    vector3 minExt;
    vector3 maxExt;

    static bool boxesIntersect (ZNavAABB3 box1, ZNavAABB3 box2)
    {
		// using 6 splitting planes to rule out intersections.
		return box1.maxExt.x < box2.minExt.x || box1.minExt.x > box2.maxExt.x ||
			box1.maxExt.y < box2.minExt.y || box1.minExt.y > box2.maxExt.y ||
			box1.maxExt.z < box2.minExt.z || box1.minExt.z > box2.maxExt.z ? false : true;
    }

    double min(double a, double b)
    {
        return a < b ? a : b;
    }

    double max(double a, double b)
    {
        return a > b ? a : b;
    }

    void clear()
    {
        self.minExt = ( -INT.Max, -INT.Max, -INT.Max);
        self.maxExt = ( INT.Max, INT.Max, INT.Max);
    }

    void setFromActor(actor mo)
    {
        if (!mo) return;
        self.minExt = ( mo.pos.x - mo.radius, mo.pos.y - mo.radius, mo.pos.z);
        self.maxExt = ( mo.pos.x + mo.radius, mo.pos.y + mo.radius, mo.pos.z + mo.height);
    }

    void expandByVector(vector3 v)
    {
        self.minExt = ( min(self.minExt.x, v.x), min(self.minExt.y, v.y), min(self.minExt.z, v.z));
        self.maxExt = ( max(self.maxExt.x, v.x), max(self.maxExt.y, v.y), max(self.maxExt.z, v.z));
    }

    bool containsPointZ(vector3 pt)
    {
        return pt.z < self.minExt.z || pt.z > self.maxExt.z ? false : true;
    }

    bool containsPoint(vector3 pt)
    {
        return pt.x < self.minExt.x || pt.x > self.maxExt.x ||
            pt.y < self.minExt.y || pt.y > self.maxExt.y ||
            pt.z < self.minExt.z || pt.z > self.maxExt.z ? false : true;
    }

    bool containsBox( ZNavAABB3 box ) 
    {
		return self.minExt.x <= box.minExt.x && box.maxExt.x <= self.maxExt.x &&
			self.minExt.y <= box.minExt.y && box.maxExt.y <= self.maxExt.y &&
			self.minExt.z <= box.minExt.z && box.maxExt.z <= self.maxExt.z;
	}

    bool intersectsAABB3 (ZNavAABB3 box)
    {
        return ZNavAABB3.boxesIntersect(self, box);
    }

    vector3 getCenter()
    {
        return (self.minExt + self.maxExt) * 0.5;
    }

    vector3 getSize()
    {
        return self.maxExt - self.minExt;
    }

    void setFromCenterAndSize (vector3 center, vector3 size)
    {
        self.minExt = ( center.x - size.x / 2, center.y - size.y / 2, center.z - size.z / 2);
        self.maxExt = ( center.x + size.x / 2, center.y + size.y / 2, center.z + size.z / 2);
    }
}

mixin class boundingbox
{
    ZNavAABB3 bbox;
}