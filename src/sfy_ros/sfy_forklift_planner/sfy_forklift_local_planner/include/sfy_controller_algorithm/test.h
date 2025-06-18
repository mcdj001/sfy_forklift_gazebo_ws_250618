/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-30 12:33:37
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-31 15:58:33
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_local_planner/include/sfy_controller_algorithm/test.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef TEST_H
#define TEST_H
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace SfyMotionController {
class FuzzyController {
public:
    // 构造函数
    FuzzyController() {
        // 初始化模糊集合
        initFuzzySets();
    }

    // 计算模糊控制输出
    double calculateK(double e_lat, double e_yaw) {
        // 对输入值进行模糊化
        std::vector<double> u_lat_membership = fuzzify(e_lat, u_lat_sets);
        std::vector<double> u_yaw_membership = fuzzify(e_yaw, u_yaw_sets);

        // 计算规则的激活度
        std::vector<double> rule_strengths = applyRules(u_lat_membership, u_yaw_membership);

        // 计算输出值（去模糊化，质心法）
        double output = defuzzify(rule_strengths, x_k_sets);
        return output;
    }

private:
    // 模糊集合的定义
    struct FuzzySet {
        double a, b, c; // 三角模糊数的三个参数

        // 计算隶属度
        double membership(double x) const {
            if (x <= a || x >= c) return 0.0;
            if (x == b) return 1.0;
            if (x < b) return (x - a) / (b - a);
            return (c - x) / (c - b);
        }
    };

    std::vector<FuzzySet> u_lat_sets; // 横向误差的模糊集
    std::vector<FuzzySet> u_yaw_sets; // 速度的模糊集
    std::vector<FuzzySet> x_k_sets;   // 输出的模糊集

    // 初始化模糊集合
    void initFuzzySets() {
        u_lat_sets = {
            {0.0, 0.0, 0.1},  // lowest
            {0.0, 0.1, 0.2},  // lower
            {0.1, 0.2, 0.3},  // low
            {0.2, 0.3, 0.4},  // middle
            {0.3, 0.4, 0.5},  // high
            {0.4, 0.5, 0.6},  // higher
            {0.5, 0.6, 0.6}   // highest
        };

        // 速度模糊集
        u_yaw_sets = {
            {0.0, 0.0, 0.3},  // lowest
            {0.0, 0.3, 0.6},  // lower
            {0.3, 0.6, 0.9},  // low
            {0.6, 0.9, 1.2},  // middle
            {0.8, 1.2, 1.5},  // high
            {1.2, 1.5, 1.8},  // higher
            {1.5, 2.0, 2.0}   // highest
        };

        // 输出模糊集
        x_k_sets = {
            {0.0, 0.0, 0.2},  // lowest
            {0.0, 0.2, 0.4},  // very_low
            {0.2, 0.4, 0.6},  // extremely_low
            {0.4, 0.6, 0.8},  // much_lower
            {0.6, 0.8, 1.0},  // lower
            {0.8, 1.0, 1.2},  // below_average
            {1.0, 1.2, 1.4},  // average
            {1.2, 1.4, 1.6},  // above_average
            {1.4, 1.6, 1.8},  // higher
            {1.6, 1.8, 2.0},  // much_high
            {1.8, 2.0, 2.2},  // very_high
            {2.0, 2.2, 2.4},  // extremely_high
            {2.2, 2.4, 2.4}   // highest
        };
    }

    // 对输入进行模糊化
    std::vector<double> fuzzify(double value, const std::vector<FuzzySet>& fuzzy_sets) {
        std::vector<double> memberships;
        for (const auto& set : fuzzy_sets) {
            memberships.push_back(set.membership(value));
        }
        return memberships;
    }

    // 应用模糊规则
    std::vector<double> applyRules(const std::vector<double>& u_lat_membership, const std::vector<double>& u_yaw_membership) {
        std::vector<double> rule_strengths(x_k_sets.size(), 0.0);

        // 规则1: lowest & lowest -> lowest
        rule_strengths[0] = std::min(u_lat_membership[0], u_yaw_membership[0]);

        // 规则2: (lower & lowest) | (lowest & lower) -> very_low
        rule_strengths[1] = std::max(std::min(u_lat_membership[1], u_yaw_membership[0]), std::min(u_lat_membership[0], u_yaw_membership[1]));

        // 规则3: (lowest & low) | (lower & lower) | (low & lowest) -> extremely_low
        rule_strengths[2] = std::max({std::min(u_lat_membership[0], u_yaw_membership[2]),
                                      std::min(u_lat_membership[1], u_yaw_membership[1]),
                                      std::min(u_lat_membership[2], u_yaw_membership[0])});

        // 规则4: (lowest & middle) | (lower & low) | (low & lower) | (middle & lowest) -> much_lower
        rule_strengths[3] = std::max({std::min(u_lat_membership[0], u_yaw_membership[3]),
                                      std::min(u_lat_membership[1], u_yaw_membership[2]),
                                      std::min(u_lat_membership[2], u_yaw_membership[1]),
                                      std::min(u_lat_membership[3], u_yaw_membership[0])});

        // 规则5: (lowest & high) | (lower & middle) | (low & low) | (middle & lower) | (high & lowest) -> lower
        rule_strengths[4] = std::max({std::min(u_lat_membership[0], u_yaw_membership[4]),
                                      std::min(u_lat_membership[1], u_yaw_membership[3]),
                                      std::min(u_lat_membership[2], u_yaw_membership[2]),
                                      std::min(u_lat_membership[3], u_yaw_membership[1]),
                                      std::min(u_lat_membership[4], u_yaw_membership[0])});

        // 规则6: (lowest & higher) | (lower & high) | (low & middle) | (middle & low) | (high & lower) | (higher & lowest) -> below_average
        rule_strengths[5] = std::max({std::min(u_lat_membership[0], u_yaw_membership[5]),
                                      std::min(u_lat_membership[1], u_yaw_membership[4]),
                                      std::min(u_lat_membership[2], u_yaw_membership[3]),
                                      std::min(u_lat_membership[3], u_yaw_membership[2]),
                                      std::min(u_lat_membership[4], u_yaw_membership[1]),
                                      std::min(u_lat_membership[5], u_yaw_membership[0])});

        // 规则7: (lowest & highest) | (lower & higher) | (low & high) | (middle & middle) | (high & low) | (higher & lower) | (highest & lowest) -> average
        rule_strengths[6] = std::max({std::min(u_lat_membership[0], u_yaw_membership[6]),
                                      std::min(u_lat_membership[1], u_yaw_membership[5]),
                                      std::min(u_lat_membership[2], u_yaw_membership[4]),
                                      std::min(u_lat_membership[3], u_yaw_membership[3]),
                                      std::min(u_lat_membership[4], u_yaw_membership[2]),
                                      std::min(u_lat_membership[5], u_yaw_membership[1]),
                                      std::min(u_lat_membership[6], u_yaw_membership[0])});

        // 规则8: (lower & highest) | (low & higher) | (middle & high) | (high & middle) | (higher & low) | (highest & lower) -> above_average
        rule_strengths[7] = std::max({std::min(u_lat_membership[1], u_yaw_membership[6]),
                                      std::min(u_lat_membership[2], u_yaw_membership[5]),
                                      std::min(u_lat_membership[3], u_yaw_membership[4]),
                                      std::min(u_lat_membership[4], u_yaw_membership[3]),
                                      std::min(u_lat_membership[5], u_yaw_membership[2]),
                                      std::min(u_lat_membership[6], u_yaw_membership[1])});

        // 规则9: (low & highest) | (middle & higher) | (high & high) | (higher & middle) | (highest & low) -> higher
        rule_strengths[8] = std::max({std::min(u_lat_membership[2], u_yaw_membership[6]),
                                      std::min(u_lat_membership[3], u_yaw_membership[5]),
                                      std::min(u_lat_membership[4], u_yaw_membership[4]),
                                      std::min(u_lat_membership[5], u_yaw_membership[3]),
                                      std::min(u_lat_membership[6], u_yaw_membership[2])});

        // 规则10: (highest & middle) | (higher & high) | (high & higher) | (middle & highest) -> much_high
        rule_strengths[9] = std::max({std::min(u_lat_membership[6], u_yaw_membership[3]),
                                      std::min(u_lat_membership[5], u_yaw_membership[4]),
                                      std::min(u_lat_membership[4], u_yaw_membership[5]),
                                      std::min(u_lat_membership[3], u_yaw_membership[6])});

        // 规则11: (highest & high) | (higher & higher) | (high & highest) -> very_high
        rule_strengths[10] = std::max({std::min(u_lat_membership[6], u_yaw_membership[4]),
                                       std::min(u_lat_membership[5], u_yaw_membership[5]),
                                       std::min(u_lat_membership[4], u_yaw_membership[6])});

        // 规则12: (highest & higher) | (higher & highest) -> extremely_high
        rule_strengths[11] = std::max(std::min(u_lat_membership[6], u_yaw_membership[5]),
                                      std::min(u_lat_membership[5], u_yaw_membership[6]));

        // 规则13: (highest & highest) -> highest
        rule_strengths[12] = std::min(u_lat_membership[6], u_yaw_membership[6]);

        return rule_strengths;
    }

    // 去模糊化（质心法）
    double defuzzify(const std::vector<double>& rule_strengths, const std::vector<FuzzySet>& output_sets) {
        double numerator = 0.0;
        double denominator = 0.0;

        for (size_t i = 0; i < rule_strengths.size(); ++i) {
            double centroid = (output_sets[i].a + output_sets[i].b + output_sets[i].c) / 3.0;
            numerator += rule_strengths[i] * centroid;
            denominator += rule_strengths[i];
        }

        if (denominator == 0.0) return 0.0; // 防止除以零
        return numerator / denominator;
    }
};

}
#endif // FUZZY_CONTROL_H