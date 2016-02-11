#include "implicit_surface.h"

inline Vector ImplicitSurface::computeGradient(Vector const& point) const {
    double eps = 1e-6;
    double x = (f(point.x + eps, point.y, point.z) - f(point.x - eps, point.y, point.z)) / (2 * eps);
    double y = (f(point.x, point.y + eps, point.z) - f(point.x, point.y - eps, point.z)) / (2 * eps);
    double z = (f(point.x, point.y, point.z + eps) - f(point.x, point.y, point.z - eps)) / (2 * eps);
    while(x*x+y*y+z*z < 1e-6 && eps < 100) {
        eps *= 2;
        x = (f(point.x + eps, point.y, point.z) - f(point.x - eps, point.y, point.z)) / (2 * eps);
        y = (f(point.x, point.y + eps, point.z) - f(point.x, point.y - eps, point.z)) / (2 * eps);
        z = (f(point.x, point.y, point.z + eps) - f(point.x, point.y, point.z - eps)) / (2 * eps);
    }
    return Vector(x, y, z);
}

bool ImplicitSurface::intersect(const Ray& ray, IntersectionInfo& info) {
    if (boundingGeom != NULL && !boundingGeom->intersect(ray, info)) {
        return false;
    }
    double lastValue, currentValue;
    double step, minStep = 1e-4;
    double lastDistance, currentDistance;
    lastValue = currentValue = f(ray.start.x, ray.start.y, ray.start.z);
    if (maxGradient.length() > 1e-6) {
        minStep = fabs(currentValue / (maxGradient.x * ray.dir.x));
        minStep = std::min(minStep, fabs(currentValue / (maxGradient.y * ray.dir.y)));
        minStep = std::min(minStep, fabs(currentValue / (maxGradient.z * ray.dir.z)));
    }
    lastDistance = currentDistance = 0.0;
    Vector point = ray.start, gradient;
    while(currentDistance < 1e99 && fabs(currentValue) > 1e-7 && std::signbit(currentValue) == std::signbit(lastValue)) {
        gradient = computeGradient(point);
        step = fabs(currentValue / (gradient.x * ray.dir.x));
        step = std::min(step, fabs(currentValue / (gradient.y * ray.dir.y)));
        step = std::min(step, fabs(currentValue / (gradient.z * ray.dir.z)));
        step = std::max(step, minStep);
        lastDistance = currentDistance;
        lastValue = currentValue;
        currentDistance += step;
        point = ray.start + ray.dir * currentDistance;
        currentValue = f(point.x, point.y, point.z);
    }
    if (currentDistance > 1e99) { return false; }
    double middleValue, middleDistance;
    while (fabs(currentValue) > 1e-7 || std::signbit(currentValue)) {
        middleDistance = (lastDistance + currentDistance) / 2;
        point = ray.start + ray.dir * middleDistance;
        middleValue = f(point.x, point.y, point.z);
        if (std::signbit(middleValue) != std::signbit(lastValue)) {
            currentValue = middleValue;
            currentDistance = middleDistance;
        }
        else {
            lastDistance = currentDistance;
            lastValue = currentValue;
            currentValue = middleValue;
            currentDistance = middleDistance;
        }
    }
    info.distance = currentDistance;
	info.ip = ray.start + ray.dir * currentDistance;
	info.normal = computeGradient(info.ip);
	info.normal.normalize();
	info.u = info.v = 0;
	info.geom = this;
    return true;
}

double ArithmeticCalculator::valueof(string::const_iterator& c) const {
	double result = 0.0;
	double deg = 1.0;
	bool flag = false;
	while(isdigit(*c) || *c == '.') {
	    if (isdigit(*c)) {
            result *= 10;
            result += *c - '0';
            c++;
	    }
	    else { flag = true; c++; continue; }
	    if (flag) {
            deg *=10;
	    }
	}
	c--;
	return result / deg;
}

void ArithmeticCalculator::apply(char op, stack<double>& st) const {
	double rarg = st.top(); st.pop();
	double larg = st.top(); st.pop();
	switch (op) {
        case '+': st.push(larg + rarg);break;
        case '-': st.push(larg - rarg);break;
        case '*': st.push(larg * rarg);break;
        case '/': st.push(larg / rarg);break;
        case '^': st.push(pow(larg, rarg));break;
        default : st.push(0);
	}
}

bool ArithmeticCalculator::isop(string::const_iterator c) const {
	return *c == '+' || *c == '-' || *c == '*' || *c == '/' || *c == '^';
}

int ArithmeticCalculator::priority(string::const_iterator op) const {
	switch (*op) {
        case '+':
        case '-':return 1;
        case '*':
        case '/':return 2;
        case '^':return 3;
        default: return 0;
	}
}

list<pair<char, double> >& ArithmeticCalculator::to_rpn(string const& expr, list<pair<char, double> >& rpnExpr) {
	string::const_iterator p = expr.begin();
	stack<string::const_iterator> ops;

	while (p != expr.end()) {
		if (*p == '(')
			ops.push(p);
		else if (isop(p)) {
			while (!ops.empty() &&
				   priority(ops.top()) >= priority(p))
				{ rpnExpr.push_back(make_pair(*(ops.top()), 0.0)); ops.pop(); }
			ops.push(p);
		}
		else if (isdigit(*p) || *p == '.')
            rpnExpr.push_back(make_pair('\0', valueof(p)));
        else if (*p == 'x' || *p == 'X' || *p == 'y' || *p == 'Y' || *p == 'z' || *p == 'Z')
            rpnExpr.push_back(make_pair(*p, 0.0));
		else if (*p == ')') {
				while(*(ops.top()) != '(')
					{ rpnExpr.push_back(make_pair(*(ops.top()), 0.0)); ops.pop(); }
				// махаме и (
				ops.pop();
		}
		p++;
	}
	// изпразваме стека
	while (!ops.empty()) {
		rpnExpr.push_back(make_pair(*(ops.top()), 0.0)); ops.pop();
    }
	return rpnExpr;
}

double ArithmeticCalculator::operator()(double x, double y, double z) const {
	list<pair<char, double> >::const_iterator p;
	stack<double> results;

	for (p = rpn.begin(); p != rpn.end(); ++p) {
		if (p->first =='\0') { results.push(p->second); }
        else if (p->first == 'x' || p->first == 'X') { results.push(x); }
        else if (p->first == 'y' || p->first == 'Y') { results.push(y); }
        else if (p->first == 'z' || p->first == 'Z') { results.push(z); }
		else { apply(p->first, results); }
	}
	return results.top();
}
