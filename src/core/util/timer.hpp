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

#ifndef _TIMER_HPP_
#define _TIMER_HPP_

#include <chrono>

class accumulate_timer {
public:
    accumulate_timer() {
        count_ = 0;
        duration_ = 0;
    }
    void start() {
        start_ = std::chrono::high_resolution_clock::now();
    }
    void stop() {
        duration_ += std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start_).count();
        count_++;
    }
    float avg() {
        if (count_ == 0) {
            return 0.f;
        }
        return duration_ / count_;
    }
    void clear() {
        count_ = 0;
        duration_ = 0.f;
    }

private:
    std::chrono::high_resolution_clock::time_point start_;

    int count_;
    float duration_;
};

#endif