#include "CurvatureWeight.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <omp.h>
#include <iostream>
#include <chrono>

std::vector<double> CurvatureWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    const size_t n = cloud.size();
    std::vector<double> weights(n, 0.0);

    auto t0 = std::chrono::high_resolution_clock::now();

    // Use static scheduling - no per-chunk synchronization overhead
    #pragma omp parallel for schedule(static, 1024)
    for (size_t i = 0; i < n; ++i) {
        const auto& pt = cloud[i];

        // KNN search
        auto neighbors = tree.knnSearch(pt, k_);

        if (neighbors.size() < 3) {
            weights[i] = 0.0;
            continue;
        }

        // Build local neighborhood matrix
        int m = static_cast<int>(neighbors.size());
        Eigen::MatrixXd X(m, 3);
        for (int j = 0; j < m; ++j) {
            const auto& p = cloud[neighbors[j].first];
            X(j, 0) = p.x - pt.x;
            X(j, 1) = p.y - pt.y;
            X(j, 2) = p.z - pt.z;
        }

        // Center
        Eigen::Vector3d mean = X.colwise().mean();
        X.rowwise() -= mean.transpose();

        // Covariance
        Eigen::Matrix3d cov = (X.transpose() * X) / static_cast<double>(m);

        // Eigenvalue decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        if (solver.info() != Eigen::Success) {
            weights[i] = 0.0;
            continue;
        }

        Eigen::Vector3d evals = solver.eigenvalues();
        double lmin = evals[0];
        double lmax = evals[2];

        // Curvature: C = λ_min / λ_max
        if (std::abs(lmax) > 1e-10) {
            weights[i] = lmin / lmax;
        } else {
            weights[i] = 0.0;
        }

        // Progress report every 100K points
        if (i % 100000 == 0) {
            #pragma omp critical
            {
                auto now = std::chrono::high_resolution_clock::now();
                double secs = std::chrono::duration<double>(now - t0).count();
                std::cerr << "  [Curvature " << i << "/" << n
                          << " " << (i / secs / 1e6) << " Mpts/s]" << std::endl;
            }
        }
    }

    return weights;
}

double CurvatureWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    // D' = D * (1 + α * C)
    return rawDist * (1.0 + alpha * weight);
}
