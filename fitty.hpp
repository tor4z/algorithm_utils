#if 0 //examples

// 01-fitty.cpp
#include <iostream>
#include <vector>
#define FITTY_VEC std::vector
#define FITTY_ENABLE_OSTREAM
#include "fitty.hpp"


int main()
{
    std::vector<float> xs{1, 2, 3, 4};
    std::vector<float> ys{11, 21, 31, 41};
    fitty::Linear<float> f;
    f.fit(xs, ys);

    auto model{f.model()};
    std::cout << "model: ";
    for (int i = 0; i < model.size(); ++i) {
        std::cout << model.at(i) << ", ";
    }
    std::cout << "\n";

    auto pred{f.predict(xs)};
    std::cout << "pred: ";
    for (int i = 0; i < pred.size(); ++i) {
        std::cout << pred.at(i) << ", ";
    }
    std::cout << "\n";

    return 0;
}

// 01-fitty_matrix.cpp
#include <iostream>
#define FITTY_ENABLE_OSTREAM
#include "fitty.hpp"


int main()
{
    {
        std::cout << "============= matmul ============\n";
        fitty::Matrix<float> m1(2, 2);
        m1 << 1, 2, 3, 4;
        fitty::Matrix<float> m2(2, 2);
        m2 << 1, 2, 3, 4;
        std::cout << m1 << "\n";
        std::cout << m2 << "\n";
        std::cout << m1 * m2 << "\n";
        m1 *= m2;
        std::cout << m1 << "\n";
    }

    {
        std::cout << "============= num * mul ============\n";
        fitty::Matrix<float> m1(2, 2);
        m1 << 1, 2, 3, 4;
        std::cout << m1 * 3 << "\n";
        std::cout << 3 * m1 << "\n";
    }

    {
        std::cout << "============= transpose ============\n";
        fitty::Matrix<float> m(1, 4);
        m << 1, 2, 3, 4;
        std::cout << "m: " << m << "\n";
        std::cout << "m.t: " << m.t() << "\n";
    }

    {
        std::cout << "============= inv ============\n";
        fitty::Matrix<float> m(2, 2);
        m << 1, 2, 3, 4;
        std::cout << "m: " << m << "\n";
        std::cout << "m.inv: " << m.inv() << "\n";
        std::cout << "m * m.inv: " << m * m.inv() << "\n";
    }

    {
        std::cout << "============= ref ============\n";
        fitty::Matrix<float> m(4, 4);
        fitty::Matrix<float> v4(4, 1);
        v4 << 1, 2, 3, 4;
        m.col(1) = v4;
        m.row(2) = v4.t();
        std::cout << m << "\n";
        fitty::Matrix<float> m_from_ref{m.row(2)};
        std::cout << m_from_ref << "\n";
    }

    {
        std::cout << "============= qr ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 19, 72, 43,
             4, 8, 5,
             0, 9, 38;
        fitty::Matrix<float> Q;
        fitty::Matrix<float> R;
        m.qr(Q, R);

        std::cout << "m: " << m << "\n";
        std::cout << "Q: " << Q << "\n";
        std::cout << "R: " << R << "\n";
        std::cout << "QR: " << Q * R << "\n";
    }

    {
        std::cout << "============= another qr ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 0, 72, 43,
             0, 8, 5,
             0, 9, 38;
        fitty::Matrix<float> Q;
        fitty::Matrix<float> R;
        m.qr(Q, R);

        std::cout << "m: " << m << "\n";
        std::cout << "Q: " << Q << "\n";
        std::cout << "R: " << R << "\n";
        std::cout << "QR: " << Q * R << "\n";
    }

    {
        std::cout << "============= eigen ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 1, 72, 43,
             4, 8, 5,
             7, 9, 38;
        fitty::Matrix<float> eigen_vals;
        fitty::Matrix<float> eigen_vecs;
        m.eigen(eigen_vals, eigen_vecs);

        std::cout << "m: " << m << "\n";
        std::cout << "eigen_vals: " << eigen_vals << "\n";
        std::cout << "eigen_vecs: " << eigen_vecs << "\n";
    }

    {
        std::cout << "============= cholesky ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 1, 72, 43,
             4, 8, 5,
             7, 9, 38;
        auto A = m * m.t(); // make sure symatric
        fitty::Matrix<float> L;
        A.cholesky(L);

        std::cout << "A: " << A << "\n";
        std::cout << "L: " << L << "\n";
    }

    {
        std::cout << "============= lu 2x2 ============\n";
        fitty::Matrix<float> m(2, 2);
        m << 4, 3,
             6, 3;
        fitty::Matrix<float> P;
        fitty::Matrix<float> L;
        fitty::Matrix<float> U;
        m.lu(P, L, U);

        std::cout << "m: " << m << "\n";
        std::cout << "L: " << L << "\n";
        std::cout << "U: " << U << "\n";
        std::cout << "L * U: " << L * U << "\n";
    }

    {
        std::cout << "============= lu 3x3 ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 1, 72, 43,
             4, 8, 5,
             7, 9, 38;
        fitty::Matrix<float> P;
        fitty::Matrix<float> L;
        fitty::Matrix<float> U;
        m.lu(P, L, U);

        std::cout << "m: " << m << "\n";
        std::cout << "L: " << L << "\n";
        std::cout << "U: " << U << "\n";
        std::cout << "L * U: " << L * U << "\n";
    }

    {
        std::cout << "============= another lu 3x3 ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 1, 2, 3,
             2, 4, 6,
             7, 9, 38;
        fitty::Matrix<float> P;
        fitty::Matrix<float> L;
        fitty::Matrix<float> U;
        m.lu(P, L, U);

        std::cout << "m: " << m << "\n";
        std::cout << "L: " << L << "\n";
        std::cout << "U: " << U << "\n";
        std::cout << "L * U: " << L * U << "\n";
    }

    {
        std::cout << "============= ker ============\n";
        fitty::Matrix<float> m(3, 3);
        m << 1, 2, 3,
             2, 4, 6,
             7, 9, 38;
        fitty::Matrix<float> k;

        if (m.ker(k)) {
            std::cout << "Ker: " << k.t() << "\n";
            std::cout << "m * Ker: " << (m * k).t() << "\n";
        } else {
            std::cout << "No solution\n";
        }
    }
    return 0;
}

#endif // examples

#ifndef FITTY_HPP_
#define FITTY_HPP_

#include <iostream>
#ifndef FITTY_U32
#   include <cstdint>
#   define FITTY_U32 uint32_t
#endif // FITTY_U32

#ifndef FITTY_I32
#   include <cstdint>
#   define FITTY_I32 int32_t
#endif // FITTY_I32

#ifndef FITTY_USIZE
#   include <cstddef>
#   define FITTY_USIZE size_t
#endif // FITTY_USIZE

#ifndef FITTY_SQRT
#   include <cmath>
#   define FITTY_SQRT std::sqrt
#endif // FITTY_SQRT

#ifndef FITTY_ASSERT
#   include <cassert>
#   define FITTY_ASSERT assert
#endif // FITTY_ASSERT

#ifndef FITTY_MEMCPY
#   include <cstring>
#   define FITTY_MEMCPY memcpy
#endif // FITTY_MEMCPY

#ifndef FITTY_VEC
#   define FITTY_VEC fitty::Vec
#endif // FITTY_VEC

#ifndef FITTY_EPS
#   define FITTY_EPS 1.0e-8
#endif // FITTY_EPS

#ifndef FITTY_EPSf
#   define FITTY_EPSf 1.0e-6f
#endif // FITTY_EPSf

#ifdef FITTY_ENABLE_OSTREAM
#   include <ostream>
#endif // FITTY_ENABLE_OSTREAM

namespace std1 {
// Copied from c++ stl

/// remove_reference
template<typename _Tp>
struct remove_reference { typedef _Tp   type; };

template<typename _Tp>
struct remove_reference<_Tp&> { typedef _Tp   type; };

template<typename _Tp>
struct remove_reference<_Tp&&> { typedef _Tp   type; };

/**
*  @brief  Convert a value to an rvalue.
*  @param  __t  A thing of arbitrary type.
*  @return The parameter cast to an rvalue-reference to allow moving it.
*/
template<typename _Tp>
_GLIBCXX_NODISCARD
constexpr typename remove_reference<_Tp>::type&&
move(_Tp&& __t) noexcept { return static_cast<typename remove_reference<_Tp>::type&&>(__t); }
} // namespace std1


namespace fitty {

// strip prefix
// rust like
using usize = FITTY_USIZE;
using i32 = FITTY_I32;
using u32 = FITTY_U32;


template<typename T>
struct Vec
{
    Vec() : data_(nullptr), size_(0) {}
    explicit Vec(usize size) : data_(new T[size]()), size_(size)
    {
        FITTY_ASSERT(size > 0);
    }

    Vec(const Vec& other) : data_(nullptr), size_(0)
    {
        if (size_ < other.size_) {
            delete [] data_;
            data_ = new T[other.size_]();
        }
        size_ = other.size_;
        FITTY_MEMCPY(data_, other.data_, sizeof(*data_) * size_);
    }

    Vec(Vec&& other) : data_(nullptr), size_(other.size_)
    {
        delete [] data_;
        data_ = other.data_;
        other.data_ = nullptr;
        other.size_ = 0;
    }

    Vec& operator=(const Vec& other)
    {
        if (&other == this) {
            return *this;
        }

        if (size_ < other.size_) {
            delete [] data_;
            data_ = new T[other.size_]();
        }
        size_ = other.size_;
        FITTY_MEMCPY(data_, other.data_, sizeof(*data_) * size_);
        return *this;
    }

    Vec& operator=(Vec&& other)
    {
        if (&other != this) {
            delete [] data_;
            size_ = other.size_;
            data_ = other.data_;
            other.data_ = nullptr;
            other.size_ = 0;
        }
        return *this;
    }

    void resize(i32 size)
    {
        FITTY_ASSERT(size > 0 && "Bad size");

        if (size_ < size) {
            delete [] data_;
            data_ = new T[size]();
        }
        size_ = size;
    }

    ~Vec() { delete [] data_; }
    bool empty() const { return size_ == 0 || data_ == nullptr; }
    usize size() const { return size_; }
    T at(i32 i) const { FITTY_ASSERT(i >= 0 && i < size_); return data_[i]; }
    T& at(i32 i) { FITTY_ASSERT(i >= 0 && i < size_); return data_[i]; }
private:
    T* data_;
    usize size_;
}; // struct Vec

// strip prefix
// rust like
template<typename T>
using vec = FITTY_VEC<T>;

template<typename T>
struct EPS { static T value() { return 0; } };
template<>
struct EPS<float> { static float value() { return FITTY_EPSf; } };
template<>
struct EPS<double> { static double value() { return FITTY_EPS; } };

template<typename T>
struct Matrix
{
private:
    struct MatrixInitializer
    {
        explicit MatrixInitializer(Matrix* m) : m(m), idx(1) {}
        MatrixInitializer& operator,(T v) { m->at(idx++) = v; return *this; }
        Matrix* m;
        i32 idx;
    }; // struct MatrixInitializer
public:
    struct Ref
    {
        Ref& operator=(const Matrix& other);
        inline void shape_check(i32 i)  const{ FITTY_ASSERT(i >= 0 && i < row_ * col_); }
        inline void shape_check(i32 r, i32 c) const { FITTY_ASSERT(r >= 0 && r < row_ && c >= 0 && c < col_); }
        inline T at(int r, int c) const { shape_check(r, c); return m_->at((row_start_ + r) * m_->col_ + (col_start_ + c)); }
        inline T at(int i) const { return at(i / col_, i % col_); }
        inline T& at(int r, int c) { shape_check(r, c); return m_->at((row_start_ + r) * m_->col_ + (col_start_ + c)); }
        inline T& at(int i) { return at(i / col_, i % col_); }
    private:
        friend class Matrix<T>;
        Ref(Matrix<T>* m, i32 row, i32 col, i32 row_start, i32 col_start)
            : m_(m), row_(row), col_(col), row_start_(row_start), col_start_(col_start)
            { FITTY_ASSERT(row_ > 0 && col_ > 0 && col_start >= 0 && row_start >= 0 && col_start < m->col_ && row_start < m->row_); }
        Matrix<T>* m_;
        i32 row_;
        i32 col_;
        i32 row_start_;
        i32 col_start_;
    }; // struct Ref

    Matrix() : row_(0), col_(0) {}
    Matrix(i32 row, i32 col) : row_(row), col_(col)
    { FITTY_ASSERT(row > 0 && col > 0); data_.resize(row_ * col_); }
    Matrix(i32 row, i32 col, const vec<T>& data) : data_(data), row_(row), col_(col)
    { FITTY_ASSERT(row * col == data_.size()); }
    Matrix(i32 row, i32 col, vec<T>&& data) : data_(std1::move(data)), row_(row), col_(col)
    { FITTY_ASSERT(row * col == data_.size()); }
    Matrix(const Matrix& other) : data_(other.data_), row_(other.row_), col_(other.col_) {}
    Matrix(Matrix&& other) : data_(std1::move(other.data_)), row_(other.row_), col_(other.col_)
    { other.col_ = 0; other.row_ = 0; }
    Matrix(const Ref& other);
    ~Matrix() = default;

    Matrix& operator=(const Ref& other);
    Matrix& operator=(const Matrix& other);
    Matrix& operator=(Matrix&& other);
    MatrixInitializer operator<<(T v) { at(0) = v; return MatrixInitializer(this); }

    void resize(i32 r, i32 c);
    inline void shape_check(i32 i) const { FITTY_ASSERT(i >= 0 && i < size()); }
    inline void shape_check(i32 r, i32 c) const { FITTY_ASSERT(r >= 0 && r < row_ && c >= 0 && c < col_); }
    inline T at(i32 i) const { shape_check(i); return data_.at(i);}
    inline T at(i32 r, i32 c) const { shape_check(r, c); return data_.at(r * col_ + c);}
    inline T& at(i32 i) { shape_check(i); return data_.at(i);}
    inline T& at(i32 r, i32 c) { shape_check(r, c); return data_.at(r * col_ + c);}
    inline i32 rows() const { return row_; }
    inline i32 cols() const { return col_; }
    inline i32 size() const { return col_ * row_; }
    inline bool empty() const { return data_.empty(); }
    inline const vec<T>& data() const { return data_; }
    inline void set_zeros() { for (i32 i = 0; i < data_.size(); ++i) data_.at(i) = 0; }

    Matrix operator*(T scalar) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*=(T scalar);
    Matrix operator*=(const Matrix& other);
    Matrix operator+=(const Matrix& other);
    Matrix operator-=(const Matrix& other);

    Matrix row(i32 i) const;
    Matrix col(i32 i) const;
    Ref row(i32 i);
    Ref col(i32 i);
    Matrix t() const;
    Matrix diag() const;
    T norm() const;
    Matrix normalized() const;
    Matrix inv() const;
    void set_eye();
    bool inv(Matrix& m) const;
    bool ker(Matrix& m) const;
    bool qr(Matrix& q, Matrix& r) const;
    bool lu(Matrix& p, Matrix& l, Matrix& u) const;
    bool cholesky(Matrix& l) const;
    bool eigen(Matrix& values, Matrix& vectors) const;
    bool svd(Matrix& u, Matrix& sigma, Matrix& vt) const;
    static Matrix eye(i32 row, i32 col);
    static Matrix zeros(i32 row, i32 col);
private:
    vec<T> data_;
    i32 row_;
    i32 col_;
}; // struct Matrix

template<typename T>
class Fitter
{
public:
    virtual const vec<T>& model() const { return model_; }
    virtual vec<T>& model() { return model_; }

    virtual bool fit(const vec<T>& xs, const vec<T>& ys) = 0;
    virtual vec<T> predict(const vec<T>& xs) = 0;
    virtual vec<T> residuals(const vec<T>& xs, const vec<T>& ys) = 0;
private:
    vec<T> model_;
}; // class Fitter

template<typename T>
class Linear: public Fitter<T>
{
public:
    using Fitter<T>::model;

    Linear() {}
    virtual bool fit(const vec<T>& xs, const vec<T>& ys) override;
    virtual vec<T> predict(const vec<T>& xs) override;
    virtual vec<T> residuals(const vec<T>& xs, const vec<T>& ys) override;
}; // class Linear

template<typename T>
inline bool abs(T v) { return v < 0 ? -v : v; }

template<typename T>
inline bool is_0(T v) { return abs(v) <= EPS<T>::value(); }

template<typename T>
inline T pow2(T v) { return v * v; }

template<typename T>
inline T pow3(T v) { return v * v * v; }

template<typename T>
bool ransac(Fitter<T>* f, const vec<T>& xs, const vec<T>& ys,
    u32 max_iter, u32 min_samples);

template<typename Ts, typename Tm>
Matrix<Tm> operator*(Ts scalar, const Matrix<Tm>& m)
{
    return m * scalar;
}

template<typename T>
void swap_row(Matrix<T>& m, i32 r1, i32 r2)
{
    for (i32 c = 0; c < m.cols(); ++c) {
        T tmp{m.at(r1, c)};
        m.at(r1, c) = m.at(r2, c);
        m.at(r2, c) = tmp;
    }
}

template<typename T>
typename Matrix<T>::Ref& Matrix<T>::Ref::operator=(const Matrix& other)
{
    FITTY_ASSERT(row_ == other.row_ && col_ == other.col_ && "Bad shape");
    for (i32 r = 0; r < row_; ++r) {
        for (i32 c = 0; c < col_; ++c) {
            at(r, c) = other.at(r, c);
        }
    }
    return *this;
}

template<typename T>
Matrix<T>::Matrix(const Ref& other)
    : data_(other.col_ * other.row_)
    , row_(other.row_)
    , col_(other.col_)
{
    FITTY_ASSERT(other.col_ > 0 && other.row_ > 0);
    for (i32 r = 0; r < row_; ++r) {
        for (i32 c = 0; c < col_; ++c) {
            at(r, c) = other.at(r, c);
        }
    }
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(const Ref& other)
{
    FITTY_ASSERT(other.col_ > 0 && other.row_ > 0);
    row_ = other.row_;
    col_ = other.col_;
    data_.resize(row_ * col_);
    for (i32 r = 0; r < row_; ++r) {
        for (i32 c = 0; c < col_; ++c) {
            at(r, c) = other.at(r, c);
        }
    }
    return *this;
}

template<typename T>
Matrix<T> Matrix<T>::row(i32 i) const
{
    FITTY_ASSERT(i >= 0 && i < row_ && "Bad argument, out of range");
    Matrix<T> result(1, col_);
    for (i32 j = 0; j < col_; ++j) {
        result.at(j) = at(i, j);
    }
    return result;
}

template<typename T>
Matrix<T> Matrix<T>::col(i32 i) const
{
    FITTY_ASSERT(i >= 0 && i < col_ && "Bad argument, out of range");
    Matrix<T> result(row_, 1);
    for (i32 j = 0; j < row_; ++j) {
        result.at(j) = at(j, i);
    }
    return result;
}

template<typename T>
typename Matrix<T>::Ref Matrix<T>::row(i32 i)
{
    FITTY_ASSERT(i >= 0 && i < row_ && "Bad argument, out of range");
    return Ref(this, 1, col_, i, 0);
}

template<typename T>
typename Matrix<T>::Ref Matrix<T>::col(i32 i)
{
    FITTY_ASSERT(i >= 0 && i < col_ && "Bad argument, out of range");
    return Ref(this, row_, 1, 0, i);
}

template<typename T>
void Matrix<T>::set_eye()
{ 
    FITTY_ASSERT(row_ == col_);
    for (i32 r = 0; r < row_; ++r) {
        for (i32 c = 0; c < col_; ++c) {
            at(r, c) = r == c ? 1 : 0;
        }
    }
}

template<typename T>
void Matrix<T>::resize(i32 r, i32 c)
{
    row_ = r;
    col_ = c;
    data_.resize(r * c);
}

template<typename T>
Matrix<T> Matrix<T>::diag() const
{
    FITTY_ASSERT(row_ == col_);

    Matrix<T> out(row_, 1);
    for (i32 i = 0; i < row_; ++i) {
        out.at(i) = at(i, i);
    }
    return out;
}

template<typename T>
T Matrix<T>::norm() const
{
    FITTY_ASSERT(row_ > 0 && col_ > 0 && "Invalid matrix");
    FITTY_ASSERT((row_ == 1 || col_ == 1) && "Not supported matrix shape");
    T sq_sum{0};
    for (i32 i = 0; i < size(); ++i) {
        sq_sum += pow2(at(i));
    }
    return FITTY_SQRT(sq_sum);
}

template<typename T>
Matrix<T> Matrix<T>::normalized() const
{
    const T n{norm()};
    const auto m_size{size()};

    Matrix<T> result(*this);
    if (!is_0(n)) {
        for (i32 i = 0; i < m_size; ++i) {
            result.at(i) = result.at(i) / n;
        }
    }
    return result;
}

template<typename T>
bool Matrix<T>::lu(Matrix& P, Matrix& L, Matrix& U) const
{
    FITTY_ASSERT(row_ > 0 && col_ > 0);
    U = *this;
    L = Matrix<T>::eye(row_, row_);
    P = Matrix<T>::eye(row_, row_);
    Matrix<T> row_op(row_, row_);
    Matrix<T> p_op(row_, row_);
    if (row_ == 1 || col_ == 1) {
        return true;
    }

    for (i32 c = 0; c < col_; ++c) {
        row_op.set_eye();
        p_op.set_eye();
        T pivot_c{U.at(c, c)};
        if (is_0(pivot_c)) {
            // swap row
            for (i32 r = c + 1; r < row_; ++r) {
                if (!is_0(U.at(r, c))) {
                    swap_row(U, c, r);
                    swap_row(p_op, c, r);
                }
            }
            P *= p_op;
            row_op = p_op * row_op;
        }

        pivot_c = U.at(c, c);
        if (is_0(pivot_c)) {
            continue;
        }

        for (i32 r = c + 1; r < row_; ++r) {
            T uv{-U.at(r, c) / pivot_c};
            T lv{U.at(r, c) / pivot_c};
            for (i32 c1 = 0; c1 < row_op.cols(); ++c1) {
                row_op.at(r, c1) += lv * row_op.at(c, c1);
                U.at(r, c1) += uv * U.at(c, c1);
            }
        }
        L *= row_op;
    }

    // row permutation
    L = P * L;
    return true;
}

//
// Taboga, Marco (2021). "QR decomposition", Lectures on matrix algebra.
// https://www.statlect.com/matrix-algebra/QR-decomposition.
//
template<typename T>
bool Matrix<T>::qr(Matrix& Q, Matrix& R) const
{
    FITTY_ASSERT(col_ > 1 && row_ > 1 && "Invalid shape for qr");

    Q.resize(row_, col_);
    R.resize(col_, col_);
    R.set_zeros();

    Matrix<T> this_col(row_, 1);
    Matrix<T> this_qc(row_, 1);
    Matrix<T> tmp(row_, 1);
    for (i32 c = 0; c < col_; ++c) {
        this_col = col(c);
        this_qc.set_zeros();
        for (i32 qc = c - 1; qc >= 0; --qc) {
            tmp = Q.col(qc);
            this_qc = this_qc - tmp * (this_col.t() * Q.col(qc)).at(0) * -1;
        }
        this_qc = this_col - this_qc;
        auto qc_norm{this_qc.norm()};
        R.at(c, c) = qc_norm;
        if (is_0(qc_norm)) {
            qc_norm = 1;
            this_qc.at(0) = 1;
        }
        Q.col(c) = this_qc * (1 / qc_norm);
    }

    for (i32 r = 0; r < col_; ++r) {
        for (i32 c = r + 1; c < col_; ++c) {
            this_qc = Q.col(r);
            tmp = col(c);
            R.at(r, c) = (this_qc.t() * tmp).at(0);
        }
    }
    return true;
}

template<typename T>
bool Matrix<T>::cholesky(Matrix& L) const
{
    FITTY_ASSERT(row_ > 0);
    FITTY_ASSERT(row_ == col_ && "Not a square matrix");

    T sum{0};
    L.resize(row_, col_);
    L.set_zeros();
    for (i32 i = 0; i < row_; ++i) {
        sum = 0;
        for (i32 k = 0; k < i; ++k) {
            sum += pow2(L.at(i, k));
        }
        L.at(i, i) = FITTY_SQRT(at(i, i) - sum);

        for (i32 j = i + 1; j < row_; ++j) {
            sum = 0;
            for (i32 k = 0; k < i; ++k) {
                sum += L.at(i, k) * L.at(j, k);
            }
            L.at(j, i) = 1 / L.at(i, i) * (at(j, i) - sum);
        }
    }

    return true;
}

template<typename T>
bool Matrix<T>::eigen(Matrix& values, Matrix& vectors) const
{
    FITTY_ASSERT(row_ > 0 && col_ > 0);
    FITTY_ASSERT(row_ == col_);

    Matrix<T> A(*this);
    Matrix<T> Q(row_, col_);
    Matrix<T> R(col_, col_);
    for (i32 i = 0; i < 100; ++i) {
        A.qr(Q, R);
        A = R * Q;
    }

    values = A.diag();
    return true;
}

template<typename T>
bool Matrix<T>::svd(Matrix& u, Matrix& sigma, Matrix& vt) const
{
    return false;
}

template<typename T>
Matrix<T> Matrix<T>::zeros(i32 row, i32 col)
{
    FITTY_ASSERT(row > 0 && col > 0 && "Invalid matrix shape");
    Matrix<T> out(row, col);
    out.data_.set_zeros();
    return out;    
}

template<typename T>
Matrix<T> Matrix<T>::eye(i32 row, i32 col)
{
    FITTY_ASSERT(row == col && "Invalid matrix shape");
    Matrix<T> out(row, col);
    out.set_eye();
    return out;
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix& other)
{
    data_ = other.data_;
    row_ = other.row_;
    col_ = other.col_;
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(Matrix&& other)
{
    data_ = std1::move(other.data_);
    row_ = other.row_;
    col_ = other.col_;
    other.row_ = 0;
    other.col_ = 0;
    return *this;
}

template<typename T>
bool Matrix<T>::ker(Matrix& k) const
{
    FITTY_ASSERT(row_ > 0 && col_ > 0);

    // Ax = 0, A = PLU
    // => PLUx = 0
    // => LUx = 0
    // => y = Ux
    // => Ly = 0
    Matrix<T> P;
    Matrix<T> L;
    Matrix<T> U;
    if (!lu(P, L, U)) {
        return false;
    }

    // find y
    Matrix<T> y(L.col_, 1);
    for (i32 r = 0; r < L.row_; ++r) {
        T sum{0};
        for (i32 c = 0; c <= r - 1; ++c) {
            sum += y.at(c) * L.at(r, c);
        }
        const auto L_rr_0{is_0(L.at(r, r))};
        if (is_0(sum)) {
            y.at(r) = L_rr_0 ? 1 : 0;
        } else {
            y.at(r) = L_rr_0 ? 0 : -sum / L.at(r, r);
        }
    }

    // find null space basis
    k.resize(row_, 1);
    k.set_zeros();
    for (i32 r = U.row_ - 1; r >= 0; --r) {
        T sum{0};
        for (i32 c = U.col_ - 1; c > r; --c) {
            sum += k.at(c) * U.at(r, c);
        }

        const auto U_rr_0{is_0(U.at(r, r))};
        if (is_0(sum)) {
            if (U_rr_0) {
                if (!is_0(y.at(r))) return false;
                k.at(r) = 1;
            } else {
                k.at(r) = y.at(r) / U.at(r, r);
            }
        } else {
            if (U_rr_0 && !is_0(y.at(r) - sum)) return false;
            k.at(r) = (y.at(r) - sum) / U.at(r, r);
        }
    }

    return true;
}

template<typename T>
Matrix<T> Matrix<T>::inv() const
{
    Matrix<T> out;
    FITTY_ASSERT(inv(out) && "Matrix not invertable");
    return out;
}

template<typename T>
bool Matrix<T>::inv(Matrix& inv_m) const
{
    if(row_ != col_) {
        return false;
    }

    if (row_ == 1) {
        if(is_0(at(0, 0))) {
            return false;
        }

        Matrix<T> out(row_, col_);
        out.at(0, 0) = 1 / at(0, 0);
        return true;
    }

    // Gauss Jordan algorithm
    inv_m = Matrix<T>::eye(row_, col_);
    auto tmp_m{*this};

    for (i32 c = 0; c < inv_m.cols(); ++c) {
        if (is_0(tmp_m.at(c, c))) {
            // swap row
            for (i32 r1 = c + 1; r1 < inv_m.rows(); ++r1) {
                if (!is_0(tmp_m.at(r1, c))) {
                    swap_row(tmp_m, c, r1);
                    swap_row(inv_m, c, r1);
                }
            }
        }

        auto tmp{tmp_m.at(c, c)};
        if(is_0(tmp)) {
            return false;
        }

        for (i32 c1 = 0; c1 < inv_m.cols(); ++c1) {
            tmp_m.at(c, c1) /= tmp;
            inv_m.at(c, c1) /= tmp;
        }

        for (i32 r = 0; r < inv_m.rows(); ++r) {
            if (r == c) continue;
            const auto t{tmp_m.at(r, c)};
            for (i32 c1 = 0; c1 < inv_m.cols(); ++c1) {
                inv_m.at(r, c1) -= inv_m.at(c, c1) * t;
                tmp_m.at(r, c1) -= t * tmp_m.at(c, c1);
            }
        }
    }

    return true;
}

template<typename T>
Matrix<T> Matrix<T>::t() const
{
    Matrix<T> result(col_, row_);
    for (i32 r = 0; r < row_; ++r) {
        for (i32 c = 0; c < col_; ++c) {
            result.at(c, r) = at(r, c);
        }
    }
    return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(T scalar) const
{
    const auto m_size{size()};
    Matrix<T> result(row_, col_);

    for (i32 i = 0; i < m_size; ++i) {
        result.at(i) = at(i) * scalar;
    }

    return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix& other) const
{
    // matrix shape check
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(col_ == other.row_ && "Matrix shape not matched");

    Matrix<T> result(row_, other.col_);
    for (i32 r = 0; r < result.row_; ++r) {
        for (i32 c = 0; c < result.col_; ++c) {
            T value{0};
            for (i32 k = 0; k < col_; ++k) {
                value += at(r, k) * other.at(k, c);
            }
            result.at(r, c) = value;
        }
    }
    return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix& other) const
{
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(row_ == other.row_ && col_ == other.col_ && "Matrix shape not matched");

    const auto m_size{size()};
    Matrix<T> result(row_, col_);
    for (i32 i = 0; i < m_size; ++i) {
        result.at(i) = at(i) + other.at(i);
    }
    return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix& other) const
{
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(row_ == other.row_ && col_ == other.col_ && "Matrix shape not matched");

    const auto m_size{size()};
    Matrix<T> result(row_, col_);
    for (i32 i = 0; i < m_size; ++i) {
        result.at(i) = at(i) - other.at(i);
    }
    return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator*=(T scalar)
{
    const auto m_size{size()};
    for (i32 i = 0; i < m_size; ++i) {
        at(i) *= scalar;
    }
    return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator*=(const Matrix& other)
{
    // matrix shape check
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(col_ == other.row_ && "Matrix shape not matched");

    Matrix<T> act_row(other.col_, 1);   // active row
    for (i32 r = 0; r < row_; ++r) {
        act_row = row(r);
        for (i32 c = 0; c < other.col_; ++c) {
            T value{0};
            for (i32 k = 0; k < col_; ++k) {
                value += act_row.at(k) * other.at(k, c);
            }
            at(r, c) = value;
        }
    }
    return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator+=(const Matrix& other)
{
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(row_ == other.row_ && col_ == other.col_ && "Matrix shape not matched");

    const auto m_size{size()};
    for (i32 i = 0; i < m_size; ++i) {
        at(i) += other.at(i);
    }
    return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator-=(const Matrix& other)
{
    FITTY_ASSERT(!empty());
    FITTY_ASSERT(row_ == other.row_ && col_ == other.col_ && "Matrix shape not matched");

    const auto m_size{size()};
    for (i32 i = 0; i < m_size; ++i) {
        at(i) -= other.at(i);
    }
    return *this;
}

template<typename T>
bool Linear<T>::fit(const vec<T>& xs, const vec<T>& ys)
{
    FITTY_ASSERT(ys.size() > 0);
    FITTY_ASSERT(xs.size() > 0);
    FITTY_ASSERT(xs.size() % ys.size() == 0);
    const usize model_dim{1 + xs.size() / ys.size()};
    model().resize(model_dim);

    usize ys_size{ys.size()};
    Matrix<T> A(ys_size, model_dim);
    Matrix<T> b(ys_size, 1, ys);
    for (usize i = 0; i < ys_size; ++i) {
        A.at(i, 0) = 1;
        for (usize j = 0; j < model_dim - 1; ++j) {
            A.at(i, j + 1) = xs.at(i * (model_dim - 1) + j);
        }
    }

    auto At{A.t()};
    Matrix<T> AtA_inv;
    if ((At * A).inv(AtA_inv)) {
        auto x = AtA_inv * At * b;
        model() = x.data();
        return true;
    }

    return false;
}

template<typename T>
vec<T> Linear<T>::predict(const vec<T>& xs)
{
    FITTY_ASSERT(!xs.empty() && "Bad argument.");
    FITTY_ASSERT(!model().empty() && "Model not defined yet.");
    const auto model_dim{model().size()};
    const auto model_dim_no_off{model().size() - 1};
    FITTY_ASSERT(xs.size() % model_dim_no_off == 0 && "Bad argument");
    const auto num_pred{xs.size() / model_dim_no_off};

    vec<T> result(num_pred);
    for (i32 i = 0; i < num_pred; ++i) {
        result.at(i) = model().at(0);
        for (i32 j = 0; j < model_dim_no_off; ++j) {
            result.at(i) += model().at(j + 1) * xs.at(i * model_dim_no_off + j);
        }
    }
    return result;
}

template<typename T>
vec<T> Linear<T>::residuals(const vec<T>& xs, const vec<T>& ys)
{
    FITTY_ASSERT(!xs.empty() && !ys.empty() && "Bad argument");
    const auto pred(predict(xs));
    FITTY_ASSERT(pred.size() == ys.size() && "pred and gt tot matched");

    vec<T> result(ys.size());
    for (usize i = 0; i < ys.size(); ++i) {
        result.at(i) = FITTY_SQRT(pow2(ys.at(i) - pred.at(i)));
    }
    return result;
}

#ifdef FITTY_ENABLE_OSTREAM
template<typename T>
std::ostream& operator<<(std::ostream& os, const Matrix<T>& m)
{
    const auto m_size{m.size()};
    os << "Matrix<"<< m.rows() << ", " << m.cols() << ">{";
    for (i32 i = 0; i < m_size; ++i) {
        if (i % m.cols() == 0) os << "\n  ";
        os << m.at(i) << ", ";
    }
    os << "\n}";
    return os;
}
#endif // FITTY_ENABLE_OSTREAM

} // namespace fitty

#endif // FITTY_HPP_
