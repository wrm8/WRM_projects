#include <gtest/gtest.h>
#include "radar_pkg/clustering/dbscan.hpp"

using namespace radar_pkg;

TEST(DBSCANTest, SimpleClustering) {
    DBSCAN::Config config;
    config.eps = 0.5;
    config.min_samples = 2;
    
    DBSCAN dbscan(config);
    
    std::vector<double> x = {0, 1, 10, 11, 20};
    std::vector<double> y = {0, 1, 10, 11, 20};
    
    auto clusters = dbscan.cluster(x, y);
    
    // 应该有两个簇（0-1 和 2-3），点 4 是噪声
    EXPECT_EQ(clusters.size(), 2);
    EXPECT_EQ(clusters[0].size(), 2);
    EXPECT_EQ(clusters[1].size(), 2);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}