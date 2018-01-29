#include "DLibInterface.h"

#include <dlib/optimization/max_cost_assignment.h>

#include <iostream>

using namespace std;
using namespace dlib;


std::vector<long> DLibInterface::linearMatching(const cv::Mat_<int>& costMatrix) {
    if (costMatrix.rows == 0 or costMatrix.cols == 0) {
        return std::vector<long>();
    }

    matrix<int> cost(costMatrix.rows, costMatrix.cols);
    for (size_t i = 0; i < costMatrix.rows; i++) {
        for (size_t j = 0; j < costMatrix.cols; j++) {
            cost(i, j) = costMatrix(i, j);
        }
    }

    // Find out the best assignment of rows to cols based on cost matrix
    return max_cost_assignment(cost);
}
