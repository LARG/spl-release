#include "EnumeratedSolver.h"
#include <cfloat>

using namespace Eigen;

EnumeratedSolver::EnumeratedSolver(float skipCost) {
  _skipCost = skipCost;
  _count = 0;
}

void EnumeratedSolver::generate(const CostMatrix& costs) {
  Assignment assignment(costs.rows());
  bool used[costs.cols()];
  memset(used, false, costs.cols());
  generate(costs, assignment, 0, used);
}

void EnumeratedSolver::generate(const CostMatrix& costs, Assignment& assignment, int row, bool* used) {
  if(row == costs.rows()) {
    _assignments.push_back(assignment);
    return;
  }
  for(int i = 0; i < costs.cols(); i++) {
    if(used[i]) continue;
    bool assigned = false;
    if(costs(row, i) > _skipCost) {
      assignment[row] = -1;
    } else {
      assignment[row] = i;
      used[i] = true;
      assignment.count++;
      assigned = true;
    }
    generate(costs, assignment, row + 1, used);
    if(assigned) {
      used[i] = false;
      assignment.count--;
    }
  }
}

Assignment EnumeratedSolver::evaluate(const CostMatrix& costs) {
  float bestCost = FLT_MAX;
  Assignment bestA;
  for(const auto& a : _assignments) {
    if(a.count == 0) continue;
    float cost = 0;
    for(int i = 0; i < a.size(); i++) {
      if(a[i] == -1) cost += _skipCost;
      else cost += costs(i, a[i]);
    }
    if(cost < bestCost) {
      bestCost = cost;
      bestA = a;
      _count = bestA.count;
    }
  }
  return bestA;
}

Assignment EnumeratedSolver::solve(const CostMatrix& costs) {
  generate(costs);
  return evaluate(costs);
}
