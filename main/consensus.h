#ifndef CONSENSUS_H
#define CONSENSUS_H

bool check_feasibility(const Node &node, const std::vector<double> &d);
double evaluate_cost(const Node& node, const std::vector<double>& d, double rho);
Node consensus_iterate(const Node &node, double rho, std::vector<double> &d, double &cost);

#endif