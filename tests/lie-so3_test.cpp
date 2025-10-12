#define HTEST_DEFINE_MAIN
#include "htest.hpp"
#define LIE_IMPLEMENTATION
#include "lie.hpp"

using namespace lie;

HT_CASE(Matrix, transpose)
{
    Matrixf<3, 3> m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    HT_ASSERT_FALSE(m.is_skew_sym());

    Matrixf<3, 2> m2;
    m << 1, 2,
         4, 5,
         7, 8;
    HT_ASSERT_FALSE(m.is_skew_sym());

    Matrixf<3, 2> m3;
    m << 0, -2, 3,
         2, 0, -6,
         -3, 6, 0;
    HT_ASSERT_TRUE(m.is_skew_sym());
}

HT_CASE(Matrix, eq)
{
    Matrixf<3, 3> m1;
    m1 << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    Matrixf<3, 3> m2;
    m2 << 1, 2, 3,
          4, 5, 6.3,
          7, 8, 9;

    HT_ASSERT_EQ(m1, m1);
    HT_ASSERT_NE(m1, m2);
}

HT_CASE(Vector, hat)
{
     lie::Vector<float, 3> v;
     v << 1, 2, 4;
     HT_ASSERT_TRUE(v.hat().is_skew_sym());
}

HT_CASE(Vector, vee)
{
     lie::Vector<float, 3> v;
     v << 1, 2, 4;
     lie::Vector<float, 3> v_trans{(v.hat().vee())};
     HT_ASSERT_EQ(v_trans, v);
}
