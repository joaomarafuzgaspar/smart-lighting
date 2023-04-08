#ifndef CONSENSUS_H
#define CONSENSUS_H

#include <vector>

class Node {
public:
  double rho;
  double d_av, y, c, cost;
  std::vector<double> k, o, l;
  std::vector<double> d;
  int n, m, index;

  void set_node();

  void initialization();
  bool check_feasibility(const std::vector<double> &d) const;
  double evaluate_cost(const std::vector<double>& d, double rho) const; 
  Node& consensus_iterate(double rho, std::vector<double> &d, double &cost);

  void set_occupancy(int occupancy);  
  int get_occupancy();
  void set_lower_bound_Occupied(double lower_bound_Occupied);
  double get_lower_bound_Occupied();
  void set_lower_bound_Unoccupied(double lower_bound_Unoccupied);
  double get_lower_bound_Unoccupied();
  void set_lower_bound();
  double get_lower_bound();
  void set_cost(double cost);
  double get_cost();

private:
  int _occupancy;
  double _lower_bound_Occupied, _lower_bound_Unoccupied, _lower_bound, _cost;
};

#endif