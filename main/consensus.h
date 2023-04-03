#ifndef CONSENSUS_H
#define CONSENSUS_H

#include <vector>

class Node {
  double d_av, y, c, cost;
  std::vector<double> k, o, l;
  std::vector<double> d;
  int n, m, index;

  bool check_feasibility(const std::vector<double> &d) const;
  double evaluate_cost(const std::vector<double>& d, double rho) const; 
  Node& consensus_iterate(double rho, std::vector<double> &d, double &cost);
};

#endif