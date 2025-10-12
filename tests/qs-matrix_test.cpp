#include "qs.hpp"
#define HTEST_DEFINE_MAIN
#include "htest.hpp"


HT_CASE(Matrix, is_sym)
{
    qs::Matrixf<3, 3> sym_m;
    sym_m << 1, 2, 3,
             2, 4, 5,
             3, 5, 6;

    qs::Matrixf<3, 3> not_sym_m;
    not_sym_m << 1, 2, 1,
                 2, 4, 5,
                 3, 5, 6;

    HT_ASSERT_FALSE(not_sym_m.is_sym());
}

HT_CASE(Matrix, is_psd)
{
    qs::Matrixf<3, 3>  m;
    m << 8, 2, 3,
         2, 9, 5,
         3, 5, 6;

    auto eye3x3{qs::Matrixf<3, 3>::eye()};
    auto zeros3x3{qs::Matrixf<3, 3>::zeros()};

    HT_ASSERT_TRUE(m.is_pd());
    HT_ASSERT_TRUE(m.is_psd());
    HT_ASSERT_TRUE(eye3x3.is_pd())
    HT_ASSERT_TRUE(eye3x3.is_psd())
    HT_ASSERT_FALSE(zeros3x3.is_pd())
    HT_ASSERT_TRUE(zeros3x3.is_psd())
}
