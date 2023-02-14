#pragma once

#include <Eigen/Core>
#include <vector>

typedef Eigen::MatrixXf CostMatrix;

class Assignment : public Eigen::VectorXi {
  public:
    Assignment() : Eigen::VectorXi() {
      count = 0;
    }
    Assignment(int rows) : Eigen::VectorXi(rows) {
      count = 0;
    }
    int count;
};

class EnumeratedSolver {
  public:
    EnumeratedSolver(float skipCost);
    Assignment solve(const CostMatrix& costs);
    int count() { return _count; }
  private:
    void generate(const CostMatrix& costs);
    void generate(const CostMatrix& costs, Assignment& assignment, int row, bool* used);
    Assignment evaluate(const CostMatrix& costs);
    std::vector<Assignment> _assignments;
    float _skipCost;
    int _count;
};
