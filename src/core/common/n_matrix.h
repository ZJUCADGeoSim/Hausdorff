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

#ifndef _ZJUCAD_MATRIX_N_MATRIX_H_
#define _ZJUCAD_MATRIX_N_MATRIX_H_

//* Jin HUANG's small gift to Yicun

#include <zjucad/matrix/matrix.h>

namespace zjucad {
namespace matrix {

// NOTICE: only suitable for simple type.
template <typename T, size_t N>
class bounded_array {
public:
    typedef zjucad::matrix::size_type size_type;
    typedef T value_type;
    typedef const T &const_reference;
    typedef T &reference;
    typedef const T *const_pointer;
    typedef T *pointer;

    // Construction and destruction
    bounded_array() : size_(0) {}
    bounded_array(size_type size) : size_(0), data_(0) { construct(size); }
    ~bounded_array() { destroy(); }

    // Resizing
    void resize(size_type size) {
        if (size != size_) {
            destroy();
            construct(size);
        }
    }

    size_type size() const { return size_; }

    // Element access
    const_reference operator[](size_type i) const {
        assert(i >= 0 && i < size_);
        return data_[i];
    }
    reference operator[](size_type i) {
        assert(i >= 0 && i < size_);
        return data_[i];
    }

    // Assignment
    bounded_array &operator=(const bounded_array &a) {
        if (this != &a) {
            resize(a.size_);
            std::copy(a.data_, a.data_ + a.size_, data_);
        }
        return *this;
    }

    typedef const_pointer const_iterator;
    const_iterator begin() const { return data_; }
    const_iterator end() const { return data_ + size_; }
    typedef pointer iterator;
    iterator begin() { return data_; }
    iterator end() { return data_ + size_; }

    void swap(bounded_array<T, N> &a) {
        if (this != &a) {
            std::swap(size_, a.size_);
            std::swap(data_, a.data_);
        }
    }

    size_type size_;
    T data_[N];

    void construct(size_type size) {
        assert(size <= N);
        size_ = size;
    }
    void destroy(void) { size_ = 0; }
};

template <typename T, size_t N>
using n_matrix = matrix<T, default_format, bounded_array<T, N>, false>;

} // namespace matrix
} // namespace zjucad

#endif
