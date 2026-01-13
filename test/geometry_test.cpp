#include <gtest/gtest.h>
#include "adore_math/geometry/interval.hpp"
#include "adore_math/geometry/projection.hpp"
#include <map>

using namespace adore::math;

TEST(GeometryTest, IntervalOperations) {
    Interval<double> a(0.0, 10.0);
    Interval<double> b(5.0, 15.0);
    
    EXPECT_TRUE(a.overlaps(b));
    EXPECT_FALSE(a.is_empty());
    
    a.expand(b);
    EXPECT_NEAR(a.lo, 0.0, 1e-9);
    EXPECT_NEAR(a.hi, 15.0, 1e-9);
    
    std::vector<Interval<double>> list = {{0, 2}, {1, 3}, {5, 6}};
    merge_intervals_in_place(list);
    ASSERT_EQ(list.size(), 2);
    EXPECT_NEAR(list[0].hi, 3.0, 1e-9);
    EXPECT_NEAR(list[1].lo, 5.0, 1e-9);
}

struct SimplePoint {
    double x, y;
    double s = 0.0; // for map value compat
};

TEST(GeometryTest, Projection) {
    std::map<double, SimplePoint> path;
    path[0.0] = {0.0, 0.0, 0.0};
    path[10.0] = {10.0, 0.0, 10.0};
    path[20.0] = {20.0, 0.0, 20.0};
    
    SimplePoint p{5.0, 2.0};
    
    auto sl = project_to_path(path, p);
    ASSERT_TRUE(sl.has_value());
    EXPECT_NEAR(sl->s, 5.0, 1e-3);
    EXPECT_NEAR(sl->l, 2.0, 1e-3); // Left is positive
    
    SimplePoint p2{5.0, -2.0};
    auto sl2 = project_to_path(path, p2);
    EXPECT_NEAR(sl2->l, -2.0, 1e-3); // Right is negative
    
    SimplePoint p3{-5.0, 0.0}; // Before start, should clamp to 0
    auto sl3 = project_to_path(path, p3);
    EXPECT_NEAR(sl3->s, 0.0, 1e-3);
    
    SimplePoint p4{25.0, 1.0}; // After end
    auto sl4 = project_to_path(path, p4);
    EXPECT_NEAR(sl4->s, 20.0, 1e-3);
}
