#ifndef __IMPLICIT_SURFACE_H__
#define __IMPLICIT_SURFACE_H__

#include <functional>
#include <string>
#include <stack>
#include <list>
#include "geometry.h"
#include "bbox.h"
using std::string;
using std::stack;
using std::list;
using std::pair;
using std::make_pair;

struct ArithmeticCalculator {
private:
    list<pair<char, double> > rpn;

    list<pair<char, double> >& to_rpn(string const& expr, list<pair<char, double> >& rpnExpr);
    double valueof(string::const_iterator& c) const;
    void apply(char op, stack<double>& st) const;
    bool isop(string::const_iterator c) const;
    int priority(string::const_iterator op) const;

public:
    ArithmeticCalculator(string const& expr = string()) { if(!expr.empty()) to_rpn(expr, rpn); }
    double operator()(double x, double y, double z) const;
};

class ImplicitSurface : public Geometry {
    using surfaceFunction = std::function<double(double, double, double)>;
    Vector computeGradient(Vector const& point) const;
public:
    ArithmeticCalculator expr;
    surfaceFunction f;
    Vector maxGradient;
    Geometry* boundingGeom;
    ImplicitSurface(surfaceFunction _f = surfaceFunction(), Vector maxGrad = Vector(0, 0, 0), Geometry *bg = NULL) :
        f(_f), maxGradient(maxGrad), boundingGeom(bg) {}
    bool intersect(const Ray& ray, IntersectionInfo& info);

    void fillProperties(ParsedBlock& pb)
	{
	    char tmp[256];
		pb.getStringProp("expr", tmp);
		string tmps(tmp);
		expr = ArithmeticCalculator(tmps);
		f = expr;
		pb.getVectorProp("MaxGrad", &maxGradient);
		pb.getGeometryProp("boundary", &boundingGeom);
	}
};

#endif // __IMPLICIT_SURFACE_H__
