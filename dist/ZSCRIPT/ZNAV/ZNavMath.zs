Class Math {
    /*
        get the pitch between two actors
    */

    const pi = 3.1415926535;

	static double radToDeg (double rad)
	{
		return rad * (180 / Math.pi);
	}

	static double degToRad (double deg)
	{
		return deg * (Math.pi / 180);
	}

    static float getPitch(Vector3 vec1, Vector3 vec2) {
        return atan2((vec1.z - vec2.z), sqrt(  ((vec1.x - vec2.x) * (vec1.x - vec2.x)) + ((vec1.y - vec2.y) * (vec1.y - vec2.y)) ));
    }

    static double getAbsAngle(Vector3 vec1, Vector3 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);// * (180 / pi);
    }

    static double getAbsAngle2D(Vector2 vec1, Vector2 vec2) {
        return atan2 (vec2.y - vec1.y, vec2.x - vec1.x);// * (180 / pi);
    }	


	static float easeInQuad(float prog)
	{
		return prog * prog;
	}

    static float easeInOutQuint(float prog) {
		return (prog < 0.5 ) ? 16 * prog * prog * prog * prog * prog : 1 - ((-2 * prog + 2) ** 5) / 2;
	}
	
	static float easeOutCirc (float prog) {
		return sqrt(1.0 - ((prog - 1) ** 2));
	}

	static float easeInCirc (float prog) {
		return sqrt(1.0 - (prog ** 2));
	}	
	
	static float easeOutQuint(float prog) {
		return 1.0 - (1.0 - prog) ** 5;
	}
	
	static float easeOutQuad (float prog) {
		return 1.0 - (1.0 - prog) * (1.0 - prog);
	}

	static float easeInOutQuad(float prog) {
		return (prog < 0.5) ? (2.0 * prog * prog) : (1.0 - ((-2.0 * prog + 2.0) ** 2.0) / 2.0);
	}

	static float easeInOutSine(float prog) {
		return (-1.0 * ( cos(3.14 * prog) - 1.0) ) / 2.0;
	}

	static float easeOutSine(float prog) {
		return sin( (prog * 3.14) / 2.0) * 1.0;
	}
	
	static vector3 Lerp (Vector3 lSrc, Vector3 lTgt, double alpha) {
		double kdX, kdY, kdZ;
		kdX = lSrc.x + ( lTgt.x - lSrc.x) * alpha;
		kdY = lSrc.y + ( lTgt.y - lSrc.y) * alpha;
		kdZ = lSrc.z + ( lTgt.z - lSrc.z) * alpha;
		return (kdX, kdY, kdZ);
	}

	static double distanceToSquared (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x) ) + ((a.y - b.y) * (a.y - b.y)) + ((a.z - b.z) * (a.z - b.z)) ;
	}

	static double distanceToSquared2 (Vector3 a, Vector3 b) {
		return ((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y));
	}

	static double Distance3D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2));
	}

	static double Distance2D (Vector3 a, Vector3 b) {
		return sqrt (((a.x - b.x) ** 2) + ((a.y - b.y) ** 2));
	}

	static int CheapDistance (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2) + ((a.z - b.z) ** 2) ;
	}

	static int CheapDistance2D (Vector3 a, Vector3 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

	static int CheapDistance2 (vector2 a, vector2 b) {
		return ((a.x - b.x) ** 2) + ((a.y - b.y) ** 2);
	}

	static int DistancefromVector (Vector3 a, Vector3 b) {
		return sqrt(Math.CheapDistance(a,b)) ;
	}

	static int DistancefromVector2 (Vector2 a, Vector2 b) {
		return sqrt(Math.CheapDistance2(a,b));
	}

	static vector2 SnapToGrid (vector2 p, int resolution)
	{
		return ( 
			int(floor(p.x / resolution) * resolution), 
			int(floor(p.y / resolution) * resolution)
		);
	}

	static vector3 SafeUnit3(Vector3 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}
	
	static vector2 SafeUnit2(Vector2 VecToUnit)
	{
		if(VecToUnit.Length()) { VecToUnit /= VecToUnit.Length(); }
		return VecToUnit;
	}

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

	static Vector3 Splerp3 (double Time, Vector3 Start, Vector3 StartAnchor, Vector3 LastAnchor, Vector3 Last)
	{
		return (
			Math.Splerp(Time, Start.X, StartAnchor.X, LastAnchor.X, Last.X),
			Math.Splerp(Time, Start.Y, StartAnchor.Y, LastAnchor.Y, Last.Y),
			Math.Splerp(Time, Start.Z, StartAnchor.Z, LastAnchor.Z, Last.Z)
		);
	}

	static double getLinedefAngle (line ln)
	{
		return atan2(ln.delta.y, ln.delta.x) - 90.;
	}

    static double normalize(double val, double minval, double maxval)
    {
        if (maxval - minval == 0) return 1;
        return (val - minval) / (maxval - minval);
    }

	static double normalizeAngle(double d)
	{
		d = d % 360;
        if (d < 0)
        {
            d += 360.;
        }
        return d;
	}

	static play Vector3 followArc(Vector3 pt1, Vector3 pt2, double alpha, double w1 = -1024., double w2 = -1024.) 
    {
        Vector3 pt1c = pt1+ (0, 0, w1);
        Vector3 pt2c = pt2 + (0, 0, w2);

		return Math.Splerp3(alpha, pt1c, pt1, pt2, pt2c);
    }

	static bool, vector2 FindLineCircleIntersections(Vector2 center, double radius, Vector2 point1, Vector2 point2)
	{
		
		if ( Math.CircleContainsPoint(center, radius, point1) ) return true, point1;
		if ( Math.CircleContainsPoint(center, radius, point2) ) return true, point2;

		vector2 nearest;
		double dx = point2.x - point1.x;
		double dy = point2.y - point1.y;

		double lcx = center.x - point1.x;
		double lcy = center.y - point1.y;

		//  project lc onto d, resulting in vector p
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

		return ( 
			(pLen2 <= dLen2) &&
        	(((px * dx) + (py * dy)) >= 0) &&
        	Math.CircleContainsPoint(center, radius, nearest)
    	);
	}

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

	static vector3 VecFromAngle(double yaw, double pitch, double dist = 1.) {
        
        Vector3 v;
        double cah = cos(pitch);

        v.x = cos(yaw) * cah;
        v.y = sin(yaw) * cah;
        v.z = -sin(pitch);

        return v * dist;
    }

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

	static bool, vector2 pointsCrossLine(line ln, vector2 v1, vector2 v2)
	{
		bool intersected; vector2 pt;
		[intersected, pt] = Math.segmentsIntersect(ln.v1.p.x, ln.v1.p.y, ln.v2.p.x, ln.v2.p.y, v1.x, v1.y, v2.x, v2.y);
		return intersected, pt;
		
	}

	static clearscope int PointOnLineSide( Vector2 p, Line l )
    {
        if ( !l ) return 0;
        return (((p.y-l.v1.p.y)*l.delta.x+(l.v1.p.x-p.x)*l.delta.y) > double.epsilon);
    }

	static double areaTriangle(vector3 a, vector3 b, vector3 c)
    {
        return abs ( ( a.x * (b.y -c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2. );
    }

	static double triarea2 (vector3 a, vector3 b, vector3 c) 
	{
		double ax = b.x - a.x;
		double ay = b.y - a.y;
		double bx = c.x - a.x;
		double by = c.y - a.y;
		return bx * ay - ax * by;
  	}
	
	static bool vequal (vector3 a, vector3 b)
	{
		return (a.x == b.x && a.y == b.y && a.z == b.z);
	}

	static double lengthSq (vector3 v)
	{
		return v.x * v.x + v.y * v.y + v.z * v.z;
	}

	static double lengthSq2 (vector3 v)
	{
		return v.x * v.x + v.y * v.y;
	}

	static double SnapToAngle(double angle, double snap)
	{
		return floor( angle / snap) * snap;
	}

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


	// adapted from
	// https://stackoverflow.com/questions/3120357/get-closest-point-to-a-line
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

Class QuadraticBezierCurve
{
	vector3 v0, v1, v2;
	const arcLengthDivisions = 128;
	bool dirty;
	double cachedLength;

	static QuadraticBezierCurve Create()
	{
		QuadraticBezierCurve curve = new('QuadraticBezierCurve');
		return curve;
	}

	void init(vector3 v0, vector3 v1, vector3 v2)
	{
		self.v0 = v0;
		self.v1 = v1;
		self.v2 = v2;
		dirty = true;
	}

	double QuadraticBezierP0( double t, double p ) {
		double k = 1 - t;
		return k * k * p;
	}

	double QuadraticBezierP1( double t, double p ) 
	{
		return 2 * ( 1 - t ) * t * p;
	}

	double QuadraticBezierP2( double t, double p ) 
	{
		return t * t * p;
	}

	double QuadraticBezier( double t, double p0, double p1, double p2 ) 
	{
		return QuadraticBezierP0( t, p0 ) + QuadraticBezierP1( t, p1 ) + QuadraticBezierP2( t, p2 );
	}

	vector3 getPoint (double t)
	{
		vector3 point = (
			QuadraticBezier( t, v0.x, v1.x, v2.x ),
			QuadraticBezier( t, v0.y, v1.y, v2.y ),
			QuadraticBezier( t, v0.z, v1.z, v2.z )
		);

		return point;
	}

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

	void set( vector3 a, vector3 b, vector3 c )
	{
		self.a = a;
		self.b = b;
		self.c = c;
	}

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

Class Plane 
{
	vector3 normal;
	double constant;

	vector3 _vector1;
	vector3 _vector2;


	void setFromNormalAndCoplanarPoint( vector3 n, vector3 pt ) {
		normal = n;
		constant = pt dot normal;
	}

	void setFromCoplanarPoints ( vector3 a, vector3 b, vector3 c )
	{
		_vector1 = c - b;
		_vector2 = a - b;

		vector3 n = _vector1 cross _vector2;
		n = Math.SafeUnit3 ( n );

		setFromNormalAndCoplanarPoint ( n, a );
	}

	vector3 projectPoint (vector3 pt)
	{
		return normal * ( -self.distanceToPoint(pt) ) + pt;
	}

	double distanceToPoint ( vector3 pt)
	{
		return (normal dot pt) + constant;
	}
}
