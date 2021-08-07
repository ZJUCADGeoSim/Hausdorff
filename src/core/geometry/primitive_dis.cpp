/*
	State Key Lab of CAD&CG Zhejiang Unv.

	Author: 
          Yicun Zheng (3130104113@zju.edu.cn)
          Haoran Sun (hrsun@zju.edu.cn)
          Jin Huang (hj@cad.zju.edu.cn)

	Copyright (c) 2004-2021 <Jin Huang>
	All rights reserved.

	Licensed under the MIT License.
*/

#include <algorithm>
#include <iomanip>

#include "primitive_dis.hpp"

#include "core/common/conf.h"
#include "core/common/n_matrix.h"
#include "core/util/util.hpp"

template <typename V>
inline double dot3(const V &a, const V &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline bool operator==(const point_t &p1, const point_t &p2) {
    return p1[0] == p2[0] && p1[1] == p2[1] && p1[2] == p2[2];
}

using namespace std;
using namespace zjucad::matrix;
const size_t N = 3;
const double epsilon = 1e-20;

// performance record
long point_triangle_count = 0;
long triangle_triangle_count = 0;

// Vladimir J. Lumelsky,
// On fast computation of distance between line segments.
// In Information Processing Letters, no. 21, pages 55-61, 1985.
// p1q1: line1
// p2q2: line2
// c1c2: closest point pair, not valid when the two segment are parallel
// vec: vector of closest point pair
void segment_segment_sqr_dis(const point_t &p1, const point_t &q1, const point_t &p2, const point_t &q2, point_t &c1, point_t &c2, point_t &vec) {
    point_t T, temp;
    double aDa, bDb, aDb, aDt, bDt;
    point_t v1 = q1 - p1, v2 = q2 - p2;
    T = p2 - p1;
    aDa = dot3(v1, v1);
    bDb = dot3(v2, v2);
    aDb = dot3(v1, v2);
    aDt = dot3(v1, T);
    bDt = dot3(v2, T);

    double t, u;
    double denom = aDa * bDb - aDb * aDb;
    t = (aDt * bDb - bDt * aDb) / denom;

    if ((t < 0) || std::isnan(t)) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    u = (t * aDb - bDt) / bDb;
    if ((u <= 0) || std::isnan(u)) {
        c2 = p2;
        t = aDt / aDa;
        if ((t <= 0) || std::isnan(t)) {
            c1 = p1;
            vec = p2 - p1;
        } else if (t >= 1) {
            c1 = p1 + v1;
            vec = p2 - c1;
        } else {
            c1 = p1 + v1 * t;
            temp = cross(T, v1);
            vec = cross(v1, temp);
        }
    } else if (u >= 1) {
        c2 = q2;
        t = (aDb + aDt) / aDa;
        if ((t <= 0) || std::isnan(t)) {
            c1 = p1;
            vec = c2 - c1;
        } else if (t >= 1) {
            c1 = p1 + v1;
            vec = c2 - c1;
        } else {
            c1 = p1 + v1 * t;
            T = c2 - p1;
            temp = cross(T, v1);
            vec = cross(v1, temp);
        }
    } else {
        c2 = p2 + v2 * u;
        if ((t <= 0) || std::isnan(t)) {
            c1 = p1;
            temp = cross(T, v2);
            vec = cross(v2, temp);
        } else if (t >= 1) {
            c1 = p1 + v1;
            T = p2 - c1;
            temp = cross(T, v2);
            vec = cross(v2, temp);
        } else {
            c1 = p1 + v1 * t;
            vec = cross(v1, v2);
            if (dot3(vec, T) < 0) {
                vec = -vec;
            }
        }
    }
}

// use algorithm from Real-Time Collision Detection, author: Christer Ericson, page 149
double segment_segment_sqr_dis(const point_t &p1, const point_t &q1, const point_t &p2, const point_t &q2, point_t &c1, point_t &c2) {
    point_t d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    double a = dot3(d1, d1), e = dot3(d2, d2), f = dot3(d2, r);
    if (a <= epsilon && e <= epsilon) {
        c1 = p1;
        c2 = p2;
        return dot3(c1 - c2, c1 - c2);
    }

    double s, t;
    if (a <= epsilon) {
        s = 0;
        t = f / e;
        t = clamp(t, 0.0, 1.0);
    } else {
        double c = dot3(d1, r);
        if (e <= epsilon) {
            t = 0;
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            double b = dot3(d1, d2);
            double denom = a * e - b * b;

            if (denom != 0.0) {
                s = clamp((b * f - c * e) / denom, 0.0, 1.0);
            } else {
                s = 0;
            }

            double tnom = b * s + f;
            if (tnom < 0) {
                t = 0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (tnom > e) {
                t = 1;
                s = clamp((b - c) / a, 0.0, 1.0);
            } else {
                t = tnom / e;
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return dot3(c1 - c2, c1 - c2);
}

// double point_rectangle_sqr_dis(const point_t &point, const rectangle_t &prim, point_t &closest_point) {
//     assert(prim.size(1) == 3 && prim.size(2) == 4);
//     assert(false);
// }

double point_primitive_sqr_dis(const point_t &point, const primitive_t *prim, point_t &closest_point) {
    ++point_triangle_count;
    // check point dim
    assert(point.size(1) == prim->points.size(1));
    assert(prim->points.size(2) == 3);
    assert(point.size(2) == 1);
    // TODO: make this large lambda as a normal inline function.
    const auto &point_primitive_closest_point = [](const point_t &point, const primitive_t *prim, point_t &barypoint) -> void {
        barypoint = zeros<double>(3, 1);

        point_t ab(N), ac(N), ap(N);
        for (size_t i = 0; i < N; ++i) {
            ab[i] = prim->points(i, 1) - prim->points(i, 0);
            ac[i] = prim->points(i, 2) - prim->points(i, 0);
            ap[i] = point(i) - prim->points(i, 0);
        }

        const double d1 = dot3(ab, ap), d2 = dot3(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            barypoint[0] = 1;
            return;
        }

        point_t &bp = ap;
        for (size_t i = 0; i < N; ++i)
            bp(i) = point[i] - prim->points(i, 1);
        const double d3 = dot3(ab, bp), d4 = dot3(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            barypoint[1] = 1;
            return;
        }

        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            const double v = d1 / (d1 - d3);
            barypoint[0] = 1 - v;
            barypoint[1] = v;
            return;
        }

        point_t &cp = bp;
        for (size_t i = 0; i < N; ++i)
            cp(i) = point[i] - prim->points(i, 2);
        const double d5 = dot3(ab, cp);
        const double d6 = dot3(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            barypoint[2] = 1;
            return;
        }

        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            barypoint[0] = 1 - w;
            barypoint[2] = w;
            return;
        }

        const double va = d3 * d6 - d5 * d4;
        const double d43 = d4 - d3, d56 = d5 - d6;
        if (va <= 0.0 && d43 >= 0.0 && d56 >= 0.0) {
            const double w = d43 / (d43 + d56);
            barypoint[1] = 1 - w;
            barypoint[2] = w;
            return;
        }

        const double denom = 1.0 / (va + vb + vc);
        const double v = vb * denom, w = vc * denom;
        barypoint[0] = 1 - v - w;
        barypoint[1] = v;
        barypoint[2] = w;
        return;
    };

    point_t barypoint;
    point_primitive_closest_point(point, prim, barypoint);
    closest_point = prim->points * barypoint;
    barypoint = point - closest_point;
    return dot3(barypoint, barypoint);
}

double primitive_primitive_sqr_dis(const primitive_t &prim1, const primitive_t &prim2) {
    ++triangle_triangle_count;

    double min_sqr_dis = std::numeric_limits<double>::max();
    bool shown_disjoint = false;

    point_t closest_p = ones<double>(3, 1), closest_q = ones<double>(3, 1), vec = ones<double>(3, 1);
    for (size_t edge_iter1 = 0; edge_iter1 < 3; ++edge_iter1) {
        for (size_t edge_iter2 = 0; edge_iter2 < 3; ++edge_iter2) {
            segment_segment_sqr_dis(prim1.points(colon(), edge_iter1),
                                    prim1.points(colon(), (edge_iter1 + 1) % 3),
                                    prim2.points(colon(), edge_iter2),
                                    prim2.points(colon(), (edge_iter2 + 1) % 3),
                                    closest_p, closest_q,
                                    vec);
            point_t v = closest_q - closest_p;

            double sqr_dis = dot3(v, v);
            if (sqr_dis <= min_sqr_dis) {
                min_sqr_dis = sqr_dis;
                // check whether current edge pair is the closest points air
                point_t closest_pair = vec;
                point_t e = prim1.points(colon(), (edge_iter1 + 2) % 3) - closest_p;

                double a = dot3(e, closest_pair);
                e = prim2.points(colon(), (edge_iter2 + 2) % 3) - closest_q;
                double b = dot3(e, closest_pair);
                // disjoint, just return
                if ((a <= 0) && (b >= 0)) {
                    return sqr_dis;
                }

                double p = dot3(closest_pair, closest_pair);
                if (a < 0)
                    a = 0;
                if (b > 0)
                    b = 0;
                if ((p - a - b) > 0)
                    shown_disjoint = 1;
            }
        }
    }

    // If the function comes to this part, it means there is no edge pair contains the closest points, which means:
    // 1. one of the closest point is a vertex, and the other point is interior to a face
    // 2. the triangles are overlapping
    // 3. an edge of one triangle is parallel to the other's face. If case 1 and 2 are not true, then the closest points from the 9 edge pairs checks above can be taken as closest points for the triangles
    // 4. triangles were degenerate

    // check case 1
    const primitive_t *prim[2] = {&prim1, &prim2};

    for (size_t i = 0; i < 2; ++i) {
        point_t normal_s = cross(prim[i]->points(colon(), 1) - prim[i]->points(colon(), 0),
                                 prim[i]->points(colon(), 2) - prim[i]->points(colon(), 1));
        double normal_l = dot3(normal_s, normal_s);
        if (normal_l > 1e-15) {
            point_t project = ones<double>(3, 1),
                    temp = ones<double>(3, 1),
                    z = ones<double>(3, 1);
            for (size_t j = 0; j < 3; ++j) {
                temp = prim[i]->points(colon(), 0) - prim[(i + 1) % 2]->points(colon(), j);
                project(j, 0) = dot3(temp, normal_s);
            }

            int point = -1;
            if ((project(0, 0) > 0) && (project(1, 0) > 0) && (project(2, 0) > 0)) {
                point = min_element(project.begin(), project.end()) - project.begin();
            } else if ((project(0, 0) < 0) && (project(1, 0) < 0) && (project(2, 0) < 0)) {
                point = max_element(project.begin(), project.end()) - project.begin();
            }

            if (point >= 0) {
                shown_disjoint = 1;
                bool is_in = true;
                for (size_t j = 0; j < 3; ++j) {
                    temp = prim[(i + 1) % 2]->points(colon(), point) - prim[i]->points(colon(), j);
                    z = cross(normal_s, prim[i]->points(colon(), (j + 1) % 3) - prim[i]->points(colon(), j));
                    if (dot3(temp, z) <= 0) {
                        is_in = false;
                        break;
                    }
                }

                if (is_in) {
                    point_t l = normal_s * (project(point, 0) / normal_l);
                    return dot3(l, l);
                }
            }
        }
    }

    // case 1 can't be shown
    // if one of these tests showned the triangles disjoint, we assume case 3 or 4,
    // otherwise, the triangles overlap
    if (shown_disjoint) {
        return min_sqr_dis;
    } else {
        return 0;
    }
}