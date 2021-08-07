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

#ifndef _STOP_CONDITION_H
#define _STOP_CONDITION_H

class absolute_error {};
class relative_error {};

template <typename error_type>
class stop_condition {
public:
    stop_condition(double error) : error_(error) {}
    bool operator()(double L, double U) {
        return U - L < error_;
    }

private:
    double error_;
};

template <>
class stop_condition<relative_error> {
public:
    stop_condition() {}
    bool operator()(double L, double U) {
        return U - L <= (U + L) / 2 * 0.01;
    }
};

#endif