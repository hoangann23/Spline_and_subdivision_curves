#include "Spline.h"
#include <iostream>
// Implementation of Spline::Bezier, Spline::BSpline, Spline::Subdiv.
// Each of these functions have input
//    ControlCurve* control,
//    Curve* curve,
//   and some int for resolution.
// Each function should take information from control (specifically, the
// list of point positions control->P ) and add points to the 2nd curve.
// When these functions are called, curve->P is an empty vector<vec2>.
// You may use Curve::addPoint( glm::vec2 position ) to append a new point
// position to curve.

typedef std::vector<glm::vec2> vec2s;

// Some possible helper functions

// Given a list of points and t in [0,1], output Bezier point at t
glm::vec2 deCasteljau(vec2s P_in, float t){
    vec2s p = P_in;
    int n = p.size();
    for (int i = 1; i < n; i++) {
        for (int j = 0; j < n - i; j++) {
            p[j] = glm::mix(p[j], p[j+1], t);
        }
    }
    return p[0];

}

glm::vec2 lerp(glm::vec2 p, glm::vec2 q, int a, int b, float t) {
    float p_coeff = (b - t) / (b - a);
    float q_coeff = (t - a) / (b - a);

    glm::vec2 p1(p[0] * p_coeff, p[1] * p_coeff);
    glm::vec2 q1(q[0] * q_coeff, q[1] * q_coeff);

    glm::vec2 result(p1[0] + q1[0], p1[1] + q1[1]);
    return result;
}
// Given 4 points and some t in [2,3], output the BSpline point at t
glm::vec2 BSplineHelper(glm::vec2 P1, glm::vec2 P2, glm::vec2 P3, glm::vec2 P4, float t){
    float tau = t - 2;
    glm::vec4 taus = glm::vec4(float(pow(tau, 3)), float(pow(tau, 2)), tau, 1);

    glm::vec4 c1 = glm::vec4(float(-1) / float(6), float(1) / float(2), float(-1) / float(2), float(1) / float(6));
    glm::vec4 c2 = glm::vec4(float(1) / float(2), -1.f, 0.f, float(2) / float(3));
    glm::vec4 c3 = glm::vec4(float(-1) / float(2), float(1) / float(2), float(1) / float(2), float(1) / float(6));
    glm::vec4 c4 = glm::vec4(float(1) / float(6), 0.f, 0.f, 0.f);
    glm::vec4 basis = glm::vec4((taus[0] * c1[0]) + (taus[1] * c1[1]) + (taus[2] * c1[2]) + (taus[3] * c1[3]),
        (taus[0] * c2[0]) + (taus[1] * c2[1]) + (taus[2] * c2[2]) + (taus[3] * c2[3]),
        (taus[0] * c3[0]) + (taus[1] * c3[1]) + (taus[2] * c3[2]) + (taus[3] * c3[3]),
        (taus[0] * c4[0]) + (taus[1] * c4[1]) + (taus[2] * c4[2]) + (taus[3] * c4[3]));

    glm::vec2 pt1 = glm::vec2(basis[0] * P1[0], basis[0] * P1[1]);
    glm::vec2 pt2 = glm::vec2(basis[1] * P2[0], basis[1] * P2[1]);
    glm::vec2 pt3 = glm::vec2(basis[2] * P3[0], basis[2] * P3[1]);
    glm::vec2 pt4 = glm::vec2(basis[3] * P4[0], basis[3] * P4[1]);

    glm::vec2 pt = glm::vec2(pt1[0] + pt2[0] + pt3[0] + pt4[0], pt1[1] + pt2[1] + pt3[1] + pt4[1]);
    return pt;
}

// Given n points, find 2*n points that are the result of subdivision.
// vec2s SubdivLeft(vec2s P_in){}  // first n points
// vec2s SubdivRight(vec2s P_in){} // The (n+1)th to 2n-th points.

void Spline::Bezier(ControlCurve *control, Curve *curve, int resolution)
{

    for (int i = 0; i < resolution + 1; i++)
    {
        // t continuously ranges from 0 to 1
        float t = float(i) / float(resolution);
        // HW4: Your code goes here.
        // curve -> addPoint( ... );
        
        glm::vec2 pointToAdd = deCasteljau(control -> P, t);
        curve->addPoint(pointToAdd);
    }
}
void Spline::BSpline(ControlCurve *control, Curve *curve, int resolution)
{
    int n = control->size();
    if (n >= 4)
    { // We only do BSpline when there are at least 4 control points
        for (int i = 0; i < resolution + 1; i++)
        {
            // t continuously ranges from 1 to n-2
            float t = 1.f + float(n - 3) * float(i) / float(resolution);

            // HW4: Your code goes here
            int k = floor(t);
            if (k == n - 2) {
                k -= 1;
            }

            glm::vec2 pointToAdd = BSplineHelper(control->P[k-1], control->P[k], control->P[k+1], control->P[k+2], t-k+2);
            curve->addPoint(pointToAdd);
        }
    }
}
void Spline::Subdiv(ControlCurve *control, Curve *curve, int subdivLevel)
{
    // HW4: Your code goes here
    // HW4: The result of subdivision should converge to the BSpline curve.
    //      You can design a recursion.  Or you can write for loops that subdivide
    //      the correct set of curve segments at each level.
    int n = control->size();
    if (subdivLevel == 0) {
        curve->P = control->P;
    }
    else {
        if (n >= 3) {
            vec2s c_prime;

            for (int i = 0; i < n - 2; i++) {
                glm::vec2 c_first = control->P[i];
                glm::vec2 c_second = control->P[i + 1];
                glm::vec2 c_third = control->P[i + 2];

                glm::vec2 c_prime1 = (c_first * 0.5f) + (c_second * 0.5f);
                glm::vec2 c_prime2 = (c_first * (float(1) / float(8))) + (c_second * 0.75f) + (c_third * (float(1) / float(8)));

                c_prime.push_back(c_prime1);
                c_prime.push_back(c_prime2);
            }

            glm::vec2 c1 = control->P[n - 2];
            glm::vec2 c2 = control->P[n - 1];
            glm::vec2 c_primeL = (c1 * 0.5f) + (c2 * 0.5f);

            c_prime.push_back(c_primeL);

            ControlCurve* newCont = new ControlCurve();
            newCont->P = c_prime;
            subdivLevel -= 1;
            Spline::Subdiv(newCont, curve, subdivLevel);
        }
    }
}
