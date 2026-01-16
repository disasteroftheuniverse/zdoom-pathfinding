/// <summary>
/// Utility math class providing common mathematical and vector operations,
/// easing functions, interpolation, distance calculations, and geometry helpers.
/// </summary>
Class Math {
    /*
        get the pitch between two actors
    */

    /// <summary>Constant value for Pi.</summary>
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
		return deg * (Math.pi / 180);
	}

    /// <summary>Calculates the pitch (angle) between two 3D vectors.</summary>
    /// <param name="vec1">First vector.</param>
    /// <param name="vec2">Second vector.</param>
    /// <returns>Pitch angle in radians.</returns>
    static float getPitch(Vector3 vec1, Vector3 vec2) {
        return atan2((vec1.z - vec2.z), sqrt(  ((vec1.x - vec2.x) * (vec1.x - vec2.x)) + ((vec1.y - vec2.y) * (vec1.y - vec2.y)) ));
    }

    /// <summary>Computes absolute angle from vec1 to vec2 in 3D space.</summary>
    /// <param name="vec1">Origin vector.</param>
    /// <param name="vec2">Target vector.</param>
    /// <returns>Absolute angle in radians.</returns>
    static double getAbsAngle(Vector3 vec1, Vector3 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);// * (180 / pi);
    }

    /// <summary>Computes absolute 2D angle from vec1 to vec2.</summary>
    /// <param name="vec1">Origin vector.</param>
    /// <param name="vec2">Target vector.</param>
    /// <returns>Angle in radians on XY plane.</returns>
    static double getAbsAngle2D(Vector2 vec1, Vector2 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);// * (180 / pi);
    }	

    /// <summary>Quadratic easing in function.</summary>
    /// <param name="prog">Progress value [0,1].</param>
    /// <returns>Eased value.</returns>
	static float easeInQuad(float prog)
	{
		return prog * prog;
	}

    /// <summary>Quintic ease in/out function.</summary>
    static float easeInOutQuint(float prog) {
		return (prog < 0.5 ) ? 16 * prog * prog * prog * prog * prog : 1 - ((-2 * prog + 2) ** 5) / 2;
	}
	
    /// <summary>Easing out using circular function.</summary>
	static float easeOutCirc (float prog) {
		return sqrt(1.0 - ((prog - 1) ** 2));
	}

    /// <summary>Easing in using circular function.</summary>
	static float easeInCirc (float prog) {
		return sqrt(1.0 - (prog ** 2));
	}	
	
    /// <summary>Easing out using quintic function.</summary>
	static float easeOutQuint(float prog) {
		return 1.0 - (1.0 - prog) ** 5;
	}
	
    /// <summary>Easing out using quadratic function.</summary>
	static float easeOutQuad (float prog) {
		return 1.0 - (1.0 - prog) * (1.0 - prog);
	}

    /// <summary>Quadratic ease in/out function.</summary>
	static float easeInOutQuad(float prog) {
		return (prog < 0.5) ? (2.0 * prog * prog) : (1.0 - ((-2.0 * prog + 2.0) ** 2.0) / 2.0);
	}

    /// <summary>Sine-based ease in/out function.</summary>
	static float easeInOutSine(float prog) {
		return (-1.0 * ( cos(3.14 * prog) - 1.0) ) / 2.0;
	}

    /// <summary>Sine-based easing out function.</summary>
	static float easeOutSine(float prog) {
		return sin( (prog * 3.14) / 2.0) * 1.0;
	}
	
    /// <summary>Linear interpolation between two 3D vectors.</summary>
    static vector3 Lerp (Vector3 lSrc, Vector3 lTgt, double alpha) {
		double kdX, kdY, kdZ;
		kdX = lSrc.x + ( lTgt.x - lSrc.x) * alpha;
		kdY = lSrc.y + ( lTgt.y - lSrc.y) * alpha;
		kdZ = lSrc.z + ( lTgt.z - lSrc.z) * alpha;
		return (kdX, kdY, kdZ);
	}

    /// <summary>Computes squared distance between two 3D points.</summary>
	static double distanceToSquared (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x) ) + ((a.y - b.y) * (a.y - b.y)) + ((a.z - b.z) * (a.z - b.z)) ;
	}

    /// <summary>Computes squared 2D distance ignoring Z.</summary>
	static double distanceToSquared2 (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y));
	}

    /// <summary>Euclidean distance in 3D between two points.</summary>
	static double Distance3D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2));
	}

    /// <summary>Euclidean distance in 2D (XY) plane.</summary>
	static double Distance2D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2));
	}

    /// <summary>Cheap integer distance calculation in 3D (no sqrt).</summary>
	static int CheapDistance (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2) ;
	}

    /// <summary>Cheap integer distance in 2D (XY).</summary>
	static int CheapDistance2D (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

    /// <summary>Cheap integer distance for 2D vector2 types.</summary>
	static int CheapDistance2 (vector2 a, vector2 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

    /// <summary>Distance between 3D vectors using CheapDistance.</summary>
	static int DistancefromVector (Vector3 a, Vector3 b) {
		return sqrt(Math.CheapDistance(a,b)) ;
	}

    /// <summary>Distance between 2D vectors using CheapDistance2.</summary>
	static int DistancefromVector2 (Vector2 a, Vector2 b) {
		return sqrt(Math.CheapDistance2(a,b));
	}

    /// <summary>Snaps a 2D vector to a grid of given resolution.</summary>
	static vector2 SnapToGrid (vector2 p, int resolution)
	{
		return ( 
			int(floor(p.x / resolution) * resolution), 
			int(floor(p.y / resolution) * resolution)
		);
	}

    /// <summary>Normalizes a 3D vector safely.</summary>
	static vector3 SafeUnit3(Vector3 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}
	
    /// <summary>Normalizes a 2D vector safely.</summary>
	static vector2 SafeUnit2(Vector2 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}

    /// <summary>Cubic spline interpolation for a single coordinate.</summary>
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

    /// <summary>Spline interpolation in 3D space.</summary>
	static Vector3 Splerp3 (double Time, Vector3 Start, Vector3 StartAnchor, Vector3 LastAnchor, Vector3 Last)
	{
		return (
			Math.Splerp(Time, Start.X, StartAnchor.X, LastAnchor.X, Last.X),
			Math.Splerp(Time, Start.Y, StartAnchor.Y, LastAnchor.Y, Last.Y),
			Math.Splerp(Time, Start.Z, StartAnchor.Z, LastAnchor.Z, Last.Z)
		);
	}

    /// <summary>Calculates angle of a linedef.</summary>
	static double getLinedefAngle (line ln)
	{
		return atan2(ln.delta.y, ln.delta.x) - 90.;
	}

    /// <summary>Normalizes a value to [0,1] range between min and max.</summary>
	static double normalize(double val, double minval, double maxval)
    {
        if (maxval - minval == 0) return 1;
        return (val - minval) / (maxval - minval);
    }

    /// <summary>Normalizes an angle to 0-360 degrees.</summary>
	static double normalizeAngle(double d)
	{
		d = d % 360;
        if (d < 0)
        {
            d += 360.;
        }
        return d;
	}

    /// <summary>Follows an arc path between two 3D points with optional height offsets.</summary>
	static play Vector3 followArc(Vector3 pt1, Vector3 pt2, double alpha, double w1 = -1024., double w2 = -1024.) 
    {
        Vector3 pt1c = pt1+ (0, 0, w1);
        Vector3 pt2c = pt2 + (0, 0, w2);

		return Math.Splerp3(alpha, pt1c, pt1, pt2, pt2c);
    }

    /// <summary>
    /// Finds intersection points between a line segment and a circle.
    /// </summary>
    /// <param name="center">Circle center.</param>
    /// <param name="radius">Circle radius.</param>
    /// <param name="point1">First endpoint of the line segment.</param>
    /// <param name="point2">Second endpoint of the line segment.</param>
    /// <returns>
    /// Tuple containing a bool indicating whether an intersection occurs,
    /// and the nearest intersection point if one exists.
    /// </returns>
	static bool, vector2 FindLineCircleIntersections(Vector2 center, double radius, Vector2 point1, Vector2 point2)
	{
		
		if ( Math.CircleContainsPoint(center, radius, point1) ) return true, point1;
		if ( Math.CircleContainsPoint(center, radius, point2) ) return true, point2;

		vector2 nearest;
		double dx = point2.x - point1.x;
		double dy = point2.y - point1.y;

		double lcx = center.x - point1.x;
		double lcy = center.y - point1.y;

		// project lc onto d, resulting in vector p
		double dLen2 = (dx * dx) + (dy * dy);
		double px = dx;
		double py = dy;

		if (dLen2 > 0)
		{
			double dp = ((lcx * dx) + (lcy * dy)) / dLen2;
			px *= dp;
			py *= dp;
		}

		nearest = (point1.x + px, point1.y + py);
		double pLen2 = (px * px) + (py * py);

		// check if projected point is within the segment and inside the circle
		return ( 
			(pLen2 <= dLen2) &&
        	(((px * dx) + (py * dy)) >= 0) &&
        	Math.CircleContainsPoint(center, radius, nearest)
    	);
	}

    /// <summary>
    /// Determines if a 2D point is within a circle.
    /// </summary>
    /// <param name="center">Circle center.</param>
    /// <param name="radius">Circle radius.</param>
    /// <param name="pt">Point to test.</param>
    /// <returns>True if the point lies within the circle, false otherwise.</returns>
	static bool CircleContainsPoint (vector2 center, double radius, vector2 pt)
	{
		if ( ( radius > 0) && pt.x >= (center.x-radius) && pt.x <= (center.x+radius) && pt.y >= (center.y-radius) && pt.y <= (center.y+radius) )
		{
			double dx = (center.x - pt.x) * (center.x - pt.x);
			double dy = (center.y - pt.y) * (center.y - pt.y);
			return ( (dx + dy) <= (radius * radius) );
		}
		else
		{
			return false;
		}
	}

    /// <summary>
    /// Converts yaw and pitch angles to a 3D vector of given length.
    /// </summary>
    /// <param name="yaw">Rotation around the Z-axis in radians.</param>
    /// <param name="pitch">Rotation around the Y-axis in radians.</param>
    /// <param name="dist">Optional vector length, defaults to 1.</param>
    /// <returns>3D vector corresponding to the given angles and length.</returns>
	static vector3 VecFromAngle(double yaw, double pitch, double dist = 1.) {
        
        Vector3 v;
        double cah = cos(pitch);

        v.x = cos(yaw) * cah;
        v.y = sin(yaw) * cah;
        v.z = -sin(pitch);

        return v * dist;
    }

    /// <summary>
    /// Checks whether two line segments intersect and returns the intersection point if they do.
    /// </summary>
	static bool, vector2 segmentsIntersect( double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) 
	{

		// Check if none of the lines are of length 0
		if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4)) {
			return false;
		}

		double denominator = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

		// Lines are parallel
		if (denominator == 0) 
		{
			return false;
		}

		double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator;
		double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator;

		// is the intersection along the segments
		if (ua < 0 || ua > 1 || ub < 0 || ub > 1) 
		{
			return false;
		}

		// Return a object with the x and y coordinates of the intersection
		double x = x1 + ua * (x2 - x1);
		double y = y1 + ua * (y2 - y1);

		return true, (x, y);
	}

    /// <summary>
    /// Determines whether a 2D line crosses a segment defined by two points.
    /// </summary>
	static bool, vector2 pointsCrossLine(line ln, vector2 v1, vector2 v2)
	{
		bool intersected; vector2 pt;
		[intersected, pt] = Math.segmentsIntersect(ln.v1.p.x, ln.v1.p.y, ln.v2.p.x, ln.v2.p.y, v1.x, v1.y, v2.x, v2.y);
		return intersected, pt;
		
	}

    /// <summary>
    /// Determines which side of a line a point lies on.
    /// </summary>
	static clearscope int PointOnLineSide( Vector2 p, Line l )
    {
        if ( !l ) return 0;
        return (((p.y-l.v1.p.y)*l.delta.x+(l.v1.p.x-p.x)*l.delta.y) > double.epsilon);
    }

    /// <summary>
    /// Computes the area of a triangle given three points.
    /// </summary>
	static double areaTriangle(vector3 a, vector3 b, vector3 c)
    {
        return abs ( ( a.x * (b.y -c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2. );
    }

    /// <summary>
    /// Alternative 2D triangle area calculation using vectors.
    /// </summary>
	static double triarea2 (vector3 a, vector3 b, vector3 c) 
	{
		double ax = b.x - a.x;
		double ay = b.y - a.y;
		double bx = c.x - a.x;
		double by = c.y - a.y;
		return bx * ay - ax * by;
  	}
	
    /// <summary>
    /// Checks if two vectors are exactly equal (component-wise).
    /// </summary>
	static bool vequal (vector3 a, vector3 b)
	{
		return (a.x == b.x && a.y == b.y && a.z == b.z);
	}

    /// <summary>
    /// Computes squared length of a 3D vector.
    /// </summary>
	static double lengthSq (vector3 v)
	{
		return v.x * v.x + v.y * v.y + v.z * v.z;
	}

    /// <summary>
    /// Computes squared length of a 2D vector (ignoring Z).
    /// </summary>
	static double lengthSq2 (vector3 v)
	{
		return v.x * v.x + v.y * v.y;
	}

    /// <summary>
    /// Snaps an angle to the nearest multiple of snap.
    /// </summary>
	static double SnapToAngle(double angle, double snap)
	{
		return floor( angle / snap) * snap;
	}

    /// <summary>
    /// Determines if a point is inside a polygon defined by an array of vertices.
    /// </summary>
	static bool isPointInPoly (vector3 pt, in array<double> points)
	{
		int i = -1;
		bool c = false;
		int l = points.size() / 3;
		int j = l -1;
		bool q;

		for (c = false, i = -1, l = (points.size() / 3), j = l-1; ++i < l; j = i)
		{
			vector3 polyA = ( points[(i*3)], points[(i*3)+ 1], points[(i*3)+2] );
			vector3 polyB = ( points[(j*3)], points[(j*3)+ 1], points[(j*3)+2] );

			q = ((polyA.y <= pt.y && pt.y < polyB.y) || (polyB.y <= pt.y && pt.y < polyA.y)) && 
			(pt.x < (polyB.x - polyA.x) * (pt.y - polyA.y) / (polyB.y - polyA.y) + polyA.x) && (c = !c);
		}

		return q;
	}

    /// <summary>
    /// Returns the closest point on a 2D projection of a line segment to a given point.
    /// </summary>
	static vector2 getClosestPointOnLine ( vector3 a, vector3 b, vector3 p)
	{
		vector2 a_to_p = (p.x - a.x, p.y - a.y);
		vector2 a_to_b = (b.x - a.x, b.y - b.y);

		double atb2 = (a_to_b.x * a_to_b.x) + (a_to_b.y * a_to_b.y);
		double atp_dot_atb = (a_to_p.x * a_to_b.x ) + (a_to_p.y * a_to_b.y);

		double t;
		if (atb2) t = atp_dot_atb / atb2;

		return ( 
			a.x + (a_to_b.x * t), 
			a.y + (a_to_b.y * t)
		);
	}
}


/// <summary>
/// Represents a quadratic Bézier curve in 3D space.
/// Provides methods to evaluate points on the curve and calculate its length.
/// </summary>
Class QuadraticBezierCurve
{
    /// <summary>The start point of the curve.</summary>
	vector3 v0, v1, v2;

    /// <summary>Number of subdivisions used to approximate curve length.</summary>
	const arcLengthDivisions = 128;

    /// <summary>Flag indicating whether cached length needs recalculation.</summary>
	bool dirty;

    /// <summary>Cached length of the curve to avoid recalculation.</summary>
	double cachedLength;

    /// <summary>
    /// Creates a new instance of a QuadraticBezierCurve.
    /// </summary>
    /// <returns>A new QuadraticBezierCurve instance.</returns>
	static QuadraticBezierCurve Create()
	{
		QuadraticBezierCurve curve = new('QuadraticBezierCurve');
		return curve;
	}

    /// <summary>
    /// Initializes the curve with three points.
    /// </summary>
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

    /// <summary>Calculates the first basis function for a quadratic Bézier curve.</summary>
    /// <param name="t">Parameter between 0 and 1.</param>
    /// <param name="p">Coordinate value (x, y, or z).</param>
    /// <returns>Weighted contribution from point P0.</returns>
	double QuadraticBezierP0( double t, double p ) {
		double k = 1 - t;
		return k * k * p;
	}

    /// <summary>Calculates the second basis function for a quadratic Bézier curve.</summary>
    /// <param name="t">Parameter between 0 and 1.</param>
    /// <param name="p">Coordinate value (x, y, or z).</param>
    /// <returns>Weighted contribution from point P1.</returns>
	double QuadraticBezierP1( double t, double p ) 
	{
		return 2 * ( 1 - t ) * t * p;
	}

    /// <summary>Calculates the third basis function for a quadratic Bézier curve.</summary>
    /// <param name="t">Parameter between 0 and 1.</param>
    /// <param name="p">Coordinate value (x, y, or z).</param>
    /// <returns>Weighted contribution from point P2.</returns>
	double QuadraticBezierP2( double t, double p ) 
	{
		return t * t * p;
	}

    /// <summary>
    /// Calculates a single coordinate value of the quadratic Bézier curve at parameter t.
    /// </summary>
    /// <param name="t">Parameter between 0 and 1.</param>
    /// <param name="p0">Start point coordinate.</param>
    /// <param name="p1">Control point coordinate.</param>
    /// <param name="p2">End point coordinate.</param>
    /// <returns>Coordinate value at t.</returns>
	double QuadraticBezier( double t, double p0, double p1, double p2 ) 
	{
		return QuadraticBezierP0( t, p0 ) + QuadraticBezierP1( t, p1 ) + QuadraticBezierP2( t, p2 );
	}

    /// <summary>
    /// Evaluates a point on the Bézier curve at parameter t.
    /// </summary>
    /// <param name="t">Parameter between 0 and 1.</param>
    /// <returns>3D point on the curve at t.</returns>
	vector3 getPoint (double t)
	{
		vector3 point = (
			QuadraticBezier( t, v0.x, v1.x, v2.x ),
			QuadraticBezier( t, v0.y, v1.y, v2.y ),
			QuadraticBezier( t, v0.z, v1.z, v2.z )
		);

		return point;
	}

    /// <summary>
    /// Calculates the approximate length of the curve by subdividing it.
    /// </summary>
    /// <returns>The curve length as a double.</returns>
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
/// Represents a triangle in 3D space and allows finding the closest point on the triangle to a given point.
/// </summary>
Class Triangle 
{
	vector3 a;
	vector3 b;
	vector3 c;

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

    /// <summary>
    /// Sets the three vertices of the triangle.
    /// </summary>
    /// <param name="a">First vertex.</param>
    /// <param name="b">Second vertex.</param>
    /// <param name="c">Third vertex.</param>
	void set( vector3 a, vector3 b, vector3 c )
	{
		self.a = a;
		self.b = b;
		self.c = c;
	}

    /// <summary>
    /// Finds the closest point on the triangle to the given point.
    /// </summary>
    /// <param name="p">Point to check.</param>
    /// <returns>Closest point on the triangle to p.</returns>
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
/// Represents a 3D plane and provides methods for projecting points and calculating distance to the plane.
/// </summary>
Class Plane 
{
	vector3 normal;
	double constant;

	vector3 _vector1;
	vector3 _vector2;

    /// <summary>
    /// Sets the plane from a normal vector and a point on the plane.
    /// </summary>
    /// <param name="n">Normal vector of the plane.</param>
    /// <param name="pt">Point on the plane.</param>
	void setFromNormalAndCoplanarPoint( vector3 n, vector3 pt ) {
		normal = n;
		constant = pt dot normal;
	}

    /// <summary>
    /// Sets the plane from three coplanar points.
    /// </summary>
    /// <param name="a">First point.</param>
    /// <param name="b">Second point.</param>
    /// <param name="c">Third point.</param>
	void setFromCoplanarPoints ( vector3 a, vector3 b, vector3 c )
	{
		_vector1 = c - b;
		_vector2 = a - b;

		vector3 n = _vector1 cross _vector2;
		n = Math.SafeUnit3 ( n );

		setFromNormalAndCoplanarPoint ( n, a );
	}

    /// <summary>
    /// Projects a 3D point onto the plane.
    /// </summary>
    /// <param name="pt">Point to project.</param>
    /// <returns>Projected point on the plane.</returns>
	vector3 projectPoint (vector3 pt)
	{
		return normal * ( -self.distanceToPoint(pt) ) + pt;
	}

    /// <summary>
    /// Calculates signed distance from a point to the plane.
    /// </summary>
    /// <param name="pt">Point to measure distance to.</param>
    /// <returns>Signed distance from point to plane.</returns>
	double distanceToPoint ( vector3 pt)
	{
		return (normal dot pt) + constant;
	}
}
