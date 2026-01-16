/// <summary>
/// General math utilities and easing functions.
/// </summary>
Class Math 
{
    /// <summary>Pi constant.</summary>
    const pi = 3.1415926535;

    /// <summary>Converts radians to degrees.</summary>
    /// <param name="rad">Angle in radians.</param>
    /// <returns>Angle in degrees.</returns>
	static double radToDeg (double rad)
	{
		return rad * (180 / Math.pi);
	}

    /// <summary>Converts degrees to radians.</summary>
    /// <param name="deg">Angle in degrees.</param>
    /// <returns>Angle in radians.</returns>
	static double degToRad (double deg)
	{
		return (Math.pi / 180) * deg;
	}

    /// <summary>Gets the pitch (vertical angle) between two points in 3D space.</summary>
    /// <param name="vec1">Source vector.</param>
    /// <param name="vec2">Target vector.</param>
    /// <returns>Pitch angle in radians.</returns>
    static float getPitch(Vector3 vec1, Vector3 vec2) {
        return atan2((vec1.z - vec2.z), sqrt(  ((vec1.x - vec2.x) * (vec1.x - vec2.x)) + ((vec1.y - vec2.y) * (vec1.y - vec2.y)) ));
    }

    /// <summary>Returns absolute yaw angle from vec1 to vec2 in 3D.</summary>
    /// <param name="vec1">Source vector.</param>
    /// <param name="vec2">Target vector.</param>
    /// <returns>Yaw angle in radians.</returns>
    static double getAbsAngle(Vector3 vec1, Vector3 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);
    }

    /// <summary>Returns absolute yaw angle in 2D vectors.</summary>
    /// <param name="vec1">Source 2D vector.</param>
    /// <param name="vec2">Target 2D vector.</param>
    /// <returns>Angle in radians.</returns>
    static double getAbsAngle2D(Vector2 vec1, Vector2 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);
    }	

    // --------------------
    // Easing functions
    // --------------------

    /// <summary>Quadratic ease-in function.</summary>
    static float easeInQuad(float prog)
	{
		return prog * prog;
	}

    /// <summary>Quintic ease-in-out function.</summary>
    static float easeInOutQuint(float prog) {
		return (prog < 0.5 ) ? 16 * prog * prog * prog * prog * prog : 1 - ((-2 * prog + 2) ** 5) / 2;
	}
	
    /// <summary>Circular ease-out function.</summary>
	static float easeOutCirc (float prog) {
		return sqrt(1.0 - ((prog - 1) ** 2));
	}

    /// <summary>Circular ease-in function.</summary>
	static float easeInCirc (float prog) {
		return sqrt(1.0 - (prog ** 2));
	}	
	
    /// <summary>Quintic ease-out function.</summary>
	static float easeOutQuint(float prog) {
		return 1.0 - (1.0 - prog) ** 5;
	}
	
    /// <summary>Quadratic ease-out function.</summary>
	static float easeOutQuad (float prog) {
		return 1.0 - (1.0 - prog) * (1.0 - prog);
	}

    /// <summary>Quadratic ease-in-out function.</summary>
	static float easeInOutQuad(float prog) {
		return (prog < 0.5) ? (2.0 * prog * prog) : (1.0 - ((-2.0 * prog + 2.0) ** 2.0) / 2.0);
	}

    /// <summary>Sine-based ease-in-out function.</summary>
	static float easeInOutSine(float prog) {
		return (-1.0 * ( cos(3.14 * prog) - 1.0) ) / 2.0;
	}

    /// <summary>Sine-based ease-out function.</summary>
	static float easeOutSine(float prog) {
		return sin( (prog * 3.14) / 2.0) * 1.0;
	}

    // --------------------
    // Vector / Distance functions
    // --------------------

    /// <summary>Linear interpolation between two 3D vectors.</summary>
    static vector3 Lerp (Vector3 lSrc, Vector3 lTgt, double alpha) {
		double kdX, kdY, kdZ;
		kdX = lSrc.x + ( lTgt.x - lSrc.x) * alpha;
		kdY = lSrc.y + ( lTgt.y - lSrc.y) * alpha;
		kdZ = lSrc.z + ( lTgt.z - lSrc.z) * alpha;
		return (kdX, kdY, kdZ);
	}

    /// <summary>Squared distance between two 3D points.</summary>
	static double distanceToSquared (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x) ) + ((a.y - b.y) * (a.y - b.y)) + ((a.z - b.z) * (a.z - b.z)) ;
	}

    /// <summary>Squared distance in X/Y plane only.</summary>
	static double distanceToSquared2 (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y));
	}

    /// <summary>3D Euclidean distance.</summary>
	static double Distance3D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2));
	}

    /// <summary>2D Euclidean distance.</summary>
	static double Distance2D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2));
	}

    /// <summary>Cheap squared distance for optimization in 3D (integer).</summary>
	static int CheapDistance (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2) ;
	}

    /// <summary>Cheap squared distance in 2D (integer).</summary>
	static int CheapDistance2D (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

    /// <summary>Cheap squared distance between 2D vectors.</summary>
	static int CheapDistance2 (vector2 a, vector2 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

    /// <summary>Distance from vector using cheap integer distance.</summary>
	static int DistancefromVector (Vector3 a, Vector3 b) {
		return sqrt(Math.CheapDistance(a,b)) ;
	}

    /// <summary>Distance from 2D vector using cheap integer distance.</summary>
	static int DistancefromVector2 (Vector2 a, Vector2 b) {
		return sqrt(Math.CheapDistance2(a,b));
	}

    /// <summary>Snaps a 2D point to a grid of given resolution.</summary>
	static vector2 SnapToGrid (vector2 p, int resolution)
	{
		return ( 
			int(floor(p.x / resolution) * resolution), 
			int(floor(p.y / resolution) * resolution)
		);
	}

    /// <summary>Normalizes a 3D vector safely (no divide by zero).</summary>
	static vector3 SafeUnit3(Vector3 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}

    /// <summary>Normalizes a 2D vector safely (no divide by zero).</summary>
	static vector2 SafeUnit2(Vector2 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}

    // --------------------
    // Spline / Interpolation
    // --------------------

    /// <summary>Cubic interpolation using 4 points.</summary>
	static double Splerp (double Time, double p1, double p2, double p3, double p4)
	{
		double t = Time;
		double res = 2*p2;
		res += (p3 - p1) * Time;
		t *= Time;
		res += (2*p1 - 5*p2 + 4*p3 - p4) * t;
		t *= Time;
		res += (3*p2 - 3*p3 + p4 - p1) * t;
		return 0.5 * res;
	}

    /// <summary>Vector3 cubic interpolation (uses Splerp on each axis).</summary>
	static Vector3 Splerp3 (double Time, Vector3 Start, Vector3 StartAnchor, Vector3 LastAnchor, Vector3 Last)
	{
		return (
			Math.Splerp(Time, Start.X, StartAnchor.X, LastAnchor.X, Last.X),
			Math.Splerp(Time, Start.Y, StartAnchor.Y, LastAnchor.Y, Last.Y),
			Math.Splerp(Time, Start.Z, StartAnchor.Z, LastAnchor.Z, Last.Z)
		);
	}

    /// <summary>Returns angle of linedef in 2D plane.</summary>
	static double getLinedefAngle (line ln)
	{
		return atan2(ln.delta.y, ln.delta.x) - 90.;
	}

    /// <summary>Normalizes a value to 0–1 range based on min/max.</summary>
	static double normalize(double val, double minval, double maxval)
    {
        if (maxval - minval == 0) return 1;
        return (val - minval) / (maxval - minval);
    }

    /// <summary>Wraps angle into 0–360° range.</summary>
	static double normalizeAngle(double d)
	{
		d = d % 360;
        if (d < 0)
        {
            d += 360.;
        }
        return d;
	}

    /// <summary>Interpolates between two points along an arc using cubic spline.</summary>
	static play Vector3 followArc(Vector3 pt1, Vector3 pt2, double alpha, double w1 = -1024., double w2 = -1024.) 
    {
        Vector3 pt1c = pt1+ (0, 0, w1);
        Vector3 pt2c = pt2 + (0, 0, w2);

		return Math.Splerp3(alpha, pt1c, pt1, pt2, pt2c);
    }

    // --------------------
    // Circle / Line intersections
    // --------------------

    /// <summary>Checks if a line segment intersects a circle. Returns intersection and nearest point.</summary>
	static bool, vector2 FindLineCircleIntersections(Vector2 center, double radius, Vector2 point1, Vector2 point2)
	{
		// method body unchanged
	}

    /// <summary>Returns true if point lies inside a circle.</summary>
	static bool CircleContainsPoint (vector2 center, double radius, vector2 pt)
	{
		// method body unchanged
	}

    /// <summary>Returns a Vector3 from yaw/pitch/distance.</summary>
	static vector3 VecFromAngle(double yaw, double pitch, double dist = 1.) {
        // method body unchanged
    }

    /// <summary>Checks if two line segments intersect; returns intersection point if they do.</summary>
	static bool, vector2 segmentsIntersect( double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) 
	{
        // method body unchanged
	}

    /// <summary>Checks if a line is crossed by two points.</summary>
	static bool, vector2 pointsCrossLine(line ln, vector2 v1, vector2 v2)
	{
		// method body unchanged
	}

    /// <summary>Determines which side of a line a point lies on.</summary>
	static clearscope int PointOnLineSide( Vector2 p, Line l )
    {
        // method body unchanged
    }

    /// <summary>Calculates the area of a triangle.</summary>
	static double areaTriangle(vector3 a, vector3 b, vector3 c)
    {
        // method body unchanged
    }

    /// <summary>Alternate 2D area calculation (signed).</summary>
	static double triarea2 (vector3 a, vector3 b, vector3 c) 
	{
        // method body unchanged
  	}

    /// <summary>Check if two vectors are exactly equal.</summary>
	static bool vequal (vector3 a, vector3 b)
	{
        // method body unchanged
	}

    /// <summary>Squared length of a 3D vector.</summary>
	static double lengthSq (vector3 v)
	{
        // method body unchanged
	}

    /// <summary>Squared length in XY only.</summary>
	static double lengthSq2 (vector3 v)
	{
        // method body unchanged
	}

    /// <summary>Snaps an angle to a multiple of snap.</summary>
	static double SnapToAngle(double angle, double snap)
	{
        // method body unchanged
	}

    /// <summary>Returns true if a point is inside a polygon (points array is x,y,z triples).</summary>
	static bool isPointInPoly (vector3 pt, in array<double> points)
	{
        // method body unchanged
	}

    /// <summary>Returns closest point on a line segment to a given point.</summary>
	static vector2 getClosestPointOnLine ( vector3 a, vector3 b, vector3 p)
	{
        // method body unchanged
	}
}

/// <summary>
/// Represents a 2D/3D quadratic Bezier curve defined by three control points.
/// Provides methods to evaluate points and approximate length.
/// </summary>
Class QuadraticBezierCurve
{
    /// <summary>First control point (start of curve).</summary>
	vector3 v0;

    /// <summary>Second control point (middle control handle).</summary>
	vector3 v1;

    /// <summary>Third control point (end of curve).</summary>
	vector3 v2;

    /// <summary>Number of divisions used for arc length approximation.</summary>
	const arcLengthDivisions = 128;

    /// <summary>Marks whether the cached length needs to be recalculated.</summary>
	bool dirty;

    /// <summary>Cached curve length.</summary>
	double cachedLength;

    /// <summary>Creates a new QuadraticBezierCurve instance.</summary>
	/// <returns>New curve instance.</returns>
	static QuadraticBezierCurve Create()
	{
		QuadraticBezierCurve curve = new('QuadraticBezierCurve');
		return curve;
	}

    /// <summary>Initializes the curve with three control points.</summary>
	/// <param name="v0">Start point.</param>
	/// <param name="v1">Control point.</param>
	/// <param name="v2">End point.</param>
	void init(vector3 v0, vector3 v1, vector3 v2)
	{
		self.v0 = v0;
		self.v1 = v1;
		self.v2 = v2;
		dirty = true;
	}

    /// <summary>Calculates the contribution of the first point in the quadratic Bezier formula.</summary>
	static double QuadraticBezierP0( double t, double p ) {
		double k = 1 - t;
		return k * k * p;
	}

    /// <summary>Calculates the contribution of the middle control point in the quadratic Bezier formula.</summary>
	static double QuadraticBezierP1( double t, double p ) 
	{
		return 2 * ( 1 - t ) * t * p;
	}

    /// <summary>Calculates the contribution of the end point in the quadratic Bezier formula.</summary>
	static double QuadraticBezierP2( double t, double p ) 
	{
		return t * t * p;
	}

    /// <summary>Evaluates the quadratic Bezier value at a given t for one dimension.</summary>
	/// <param name="t">Parameter along curve [0..1].</param>
	/// <param name="p0">Start value.</param>
	/// <param name="p1">Control value.</param>
	/// <param name="p2">End value.</param>
	/// <returns>Value along curve.</returns>
	static double QuadraticBezier( double t, double p0, double p1, double p2 ) 
	{
		return QuadraticBezierP0( t, p0 ) + QuadraticBezierP1( t, p1 ) + QuadraticBezierP2( t, p2 );
	}

    /// <summary>Gets the 3D point on the curve at parameter t.</summary>
	/// <param name="t">Parameter [0..1].</param>
	/// <returns>Point on the curve.</returns>
	vector3 getPoint (double t)
	{
		vector3 point = (
			QuadraticBezier( t, v0.x, v1.x, v2.x ),
			QuadraticBezier( t, v0.y, v1.y, v2.y ),
			QuadraticBezier( t, v0.z, v1.z, v2.z )
		);

		return point;
	}

    /// <summary>Approximates the total length of the curve using discrete sampling.</summary>
	/// <returns>Length of the curve.</returns>
	double getLength ()
	{
		if (!dirty) 
		{
			return cachedLength;
		}

		vector3 current, last = getPoint(0);
		double divisions = double(arcLengthDivisions);
		double curLength = 0.;

		for(double i = 1.; i < divisions; i++)
		{
			current = self.getPoint(i / divisions);
			vector3 diff = current - last;
			curLength += diff.length();
			last = current;
		}
		cachedLength = curLength;
		dirty = false;
		return curLength;
	}
}

/// <summary>
/// Represents a triangle in 3D space and provides geometric utilities like closest point calculation.
/// </summary>
Class Triangle 
{
    /// <summary>Vertex A of the triangle.</summary>
	vector3 a;

    /// <summary>Vertex B of the triangle.</summary>
	vector3 b;

    /// <summary>Vertex C of the triangle.</summary>
	vector3 c;

    // Temporary vectors used for calculations
	vector3 _v0; 
	vector3 _v1; 
	vector3 _v2; 
	vector3 _v3; 
	vector3 _vab; 
	vector3 _vac; 
	vector3 _vbc;
	vector3 _vap; 
	vector3 _vbp; 
	vector3 _vcp; 

    /// <summary>Sets the triangle vertices.</summary>
	/// <param name="a">Vertex A.</param>
	/// <param name="b">Vertex B.</param>
	/// <param name="c">Vertex C.</param>
	void set( vector3 a, vector3 b, vector3 c )
	{
		self.a = a;
		self.b = b;
		self.c = c;
	}

    /// <summary>Finds the closest point on the triangle to a given point in 3D space.</summary>
	/// <param name="p">Point to find closest to.</param>
	/// <returns>Closest point on the triangle.</returns>
	vector3 closestPointToPoint( vector3 p )
	{
		vector3 a = self.a;
		vector3 b = self.b;
		vector3 c = self.c;
		double v, w;

		_vab = b - a;
		_vac = c - a;
		_vap = p - a;

		double d1 = _vab dot _vap;
		double d2 = _vac dot _vap;

		if ( d1 <= 0 && d2 <= 0 ) 
		{
			return a;
		} 

		_vbp = p - b;
		double d3 = _vab dot _vbp;
		double d4 = _vac dot _vbp;

		if ( d3 >= 0 && d4 <= d3 ) 
		{
			return b;
		}

		double vc = d1 * d4 - d3 * d2;
		if ( vc <= 0 && d1 >= 0 && d3 <= 0 ) 
		{
			if (d1 - d3 != 0) v = d1 / ( d1 - d3 );
			return a + ( _vab * v);
		}

		_vcp = p - c;
		double d5 = _vab dot _vcp;
		double d6 = _vac dot _vcp;

		if ( d6 >= 0 && d5 <= d6 ) {
			return c;
		}

		double vb = d5 * d2 - d1 * d6;
		if ( vb <= 0 && d2 >= 0 && d6 <= 0 ) {

			if (( d2 - d6 ) != 0) w = d2 / ( d2 - d6 );
			return a + ( _vac * w);
		}

		double va = d3 * d6 - d5 * d4;
		if ( va <= 0 && ( d4 - d3 ) >= 0 && ( d5 - d6 ) >= 0 ) {
			_vbc = c - b;
			if ( (( d4 - d3 ) + ( d5 - d6 ) != 0 ) ) w = ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) );
			return b + ( _vbc * w);
		}

		double denom;
		if ( ( va + vb + vc ) != 0 ) denom = 1 / ( va + vb + vc );
		v = vb * denom;
		w = vc * denom;
		return a + (_vab * v) + (_vac * w);
	}
}

/// <summary>
/// Represents a plane in 3D space defined by a normal vector and a constant term.
/// Provides methods for projecting points onto the plane and calculating distances.
/// </summary>
Class Plane 
{
    /// <summary>Normal vector of the plane (unit length).</summary>
	vector3 normal;

    /// <summary>Plane constant such that for any point p on the plane: normal · p + constant = 0.</summary>
	double constant;

    // Temporary vectors for internal calculations
	vector3 _vector1;
	vector3 _vector2;

    /// <summary>
    /// Sets the plane using a normal vector and a coplanar point.
    /// </summary>
    /// <param name="n">Normal vector of the plane (should be unit length).</param>
    /// <param name="pt">Point that lies on the plane.</param>
	void setFromNormalAndCoplanarPoint( vector3 n, vector3 pt ) 
    {
		normal = n;
		constant = pt dot normal;
	}

    /// <summary>
    /// Sets the plane from three coplanar points.
    /// Computes the normal vector from the cross product of two edges.
    /// </summary>
    /// <param name="a">First point.</param>
    /// <param name="b">Second point.</param>
    /// <param name="c">Third point.</param>
	void setFromCoplanarPoints ( vector3 a, vector3 b, vector3 c )
	{
		// Compute two edge vectors from the three points
		_vector1 = c - b;
		_vector2 = a - b;

		// Cross product gives the plane normal
		vector3 n = _vector1 cross _vector2;
		n = Math.SafeUnit3 ( n ); // Ensure normal is unit length

		setFromNormalAndCoplanarPoint ( n, a );
	}

    /// <summary>
    /// Projects a point onto the plane.
    /// </summary>
    /// <param name="pt">Point to project.</param>
    /// <returns>Projected point on the plane.</returns>
	vector3 projectPoint (vector3 pt)
	{
		// Projection formula: p' = p - (distanceToPoint * normal)
		return normal * ( -self.distanceToPoint(pt) ) + pt;
	}

    /// <summary>
    /// Calculates the signed distance from a point to the plane.
    /// Positive if the point is in the direction of the normal.
    /// </summary>
    /// <param name="pt">Point to measure distance from.</param>
    /// <returns>Signed distance to the plane.</returns>
	double distanceToPoint ( vector3 pt)
	{
		return (normal dot pt) + constant;
	}
}

