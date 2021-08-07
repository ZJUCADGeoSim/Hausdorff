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

#ifndef LOG_HELPER_
#define LOG_HELPER_

#include <iostream>
#include <string>

enum LogLevel {
    ERROR = 0,
    WARN = 1,
    INFO = 2
};

class logs {
public:
    static LogLevel _level; // global control

    logs(std::ostream &os) : os_(os) {}

    template <typename T>
    logs &operator<<(const T &v) {
        if (level_ <= _level)
            os_ << v;
        return *this;
    }

    typedef std::basic_ostream<char, std::char_traits<char>> os_t;
    typedef os_t &(*manip)(os_t &);
    logs &operator<<(manip m) {
        // call the function, but we cannot return it's value
        m(os_);
        return *this;
    }

    logs &operator<<(LogLevel l) {
        level_ = l;
        return *this;
    }

private:
    LogLevel level_;
    std::ostream &os_;
};

#endif
