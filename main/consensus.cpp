#include "consensus.h"
#include <iostream>
#include <limits>
#include <cmath>

void Node::set_node() {
  _occupancy = 0;
  _lower_bound_Occupied = 0, _lower_bound_Unoccupied = 0, _cost = 0;
  _lower_bound = _lower_bound_Unoccupied;
}

void Node::set_occupancy(int occupancy) {
  _occupancy = occupancy;
  set_lower_bound();
}

int Node::get_occupancy() {return _occupancy;}

void Node::set_lower_bound_Occupied(double lower_bound_Occupied) {
  _lower_bound_Occupied = lower_bound_Occupied;
  set_lower_bound();
}

double Node::get_lower_bound_Occupied() {return _lower_bound_Occupied;}

void Node::set_lower_bound_Unoccupied(double lower_bound_Unoccupied) {
  _lower_bound_Unoccupied = lower_bound_Unoccupied;
  set_lower_bound();
}

double Node::get_lower_bound_Unoccupied() {return _lower_bound_Unoccupied;}

void Node::set_lower_bound() {
  if (_occupancy == 1)
    _lower_bound = _lower_bound_Occupied;
  else if (_occupancy == 0)
    _lower_bound = _lower_bound_Unoccupied;
}

double Node::get_lower_bound() {return _lower_bound;}

void Node::set_cost(double cost) {_cost = cost;}

double Node::get_cost() {return _cost;}

// bool Node::check_feasibility(const std::vector<double> &d) const
// {
//     const double tol = 0.001; // tolerance for rounding errors

//     if (d[this->index] < -tol)
//     {
//         return false;
//     }

//     if (d[this->index] > 100 + tol)
//     {
//         return false;
//     }

//     double d_dot_k = 0;
//     for (size_t i = 0; i < d.size(); ++i)
//     {
//         d_dot_k += d[i] * this->k[i];
//     }

//     if (d_dot_k < this->l[this->index] - this->o[this->index] - tol)
//     {
//         return false;
//     }

//     return true;
// }

// double Node::evaluate_cost(const std::vector<double>& d, double rho) const
// {
//   double cost = 0.0;

//   for (size_t i = 0; i < d.size(); ++i) {
//       cost += this->c * d[i] + this->y * (d[i] - this->d_av);
//   }

//   double norm_squared = 0.0;
//   for (size_t i = 0; i < d.size(); ++i) {
//       norm_squared += std::pow(d[i] - this->d_av, 2);
//   }

//   cost += rho / 2 * norm_squared;

//   return cost;
// }

// Node& Node::consensus_iterate(double rho, std::vector<double> &d, double &cost)
// {
//   std::vector<double> d_best(2, -1);
//   double cost_best = 1000000; // large number
//   double z = rho * this->d_av - this->y - this->c;

//   // unconstrained minimum
//   double d_u = (1 / rho) * z;
//   if (check_feasibility({d_u}))
//   {
//       double cost_unconstrained = evaluate_cost({d_u}, rho);
//       if (cost_unconstrained < cost_best)
//       {
//           d_best = {d_u};
//           cost_best = cost_unconstrained;
//       }
//   }

//   // compute minimum constrained to linear boundary
//   double d_bl = (1 / rho) * z - this->k[this->index] / this->n * (this->o[this->index] - this->l[this->index] + (1 / rho) * z * this->k[this->index]);
//   if (check_feasibility({d_bl}))
//   {
//       double cost_boundary_linear = evaluate_cost({d_bl}, rho);
//       if (cost_boundary_linear < cost_best)
//       {
//           d_best = {d_bl};
//           cost_best = cost_boundary_linear;
//       }
//   }

//   // compute minimum constrained to 0 boundary
//   double d_b0 = (1 / rho) * z;
//   if (check_feasibility({d_b0, 0.0}))
//   {
//       double cost_boundary_0 = evaluate_cost({d_b0, 0.0}, rho);
//       if (cost_boundary_0 < cost_best)
//       {
//           d_best = {d_b0, 0.0};
//           cost_best = cost_boundary_0;
//       }
//   }

//   // compute minimum constrained to 100 boundary
//   double d_b1 = (1 / rho) * z;
//   if (check_feasibility({d_b1, 100.0}))
//   {
//       double cost_boundary_100 = evaluate_cost({d_b1, 100.0}, rho);
//       if (cost_boundary_100 < cost_best)
//       {
//           d_best = {d_b1, 100.0};
//           cost_best = cost_boundary_100;
//       }
//   }

//   // compute minimum constrained to linear and 0 boundary
//   double d_l0 = (1 / rho) * z - (1 / this->m) * this->k[this->index] * (this->o[this->index] - this->l[this->index]) + (1 / rho / this->m) * this->k[this->index] * (this->k[this->index] * z - z * this->k[this->index]);
//   if (check_feasibility({d_l0, 0.0}))
//   {
//       double cost_linear_0 = evaluate_cost({d_l0, 0.0}, rho);
//       if (cost_linear_0 < cost_best)
//       {
//           d_best = {d_l0, 0.0};
//           cost_best = cost_linear_0;
//       }
//   }

//   // compute minimum constrained to linear and 100 boundary
//   double d_l1 = (1 / rho) * z - (1 / this->m) * this->k[this->index] * (this->o[this->index] - this->l[this->index] + 100 * this->k[this->index]) + (1 / rho / this->m) * this->k[this->index] * (this->k[this->index] * z - z * this->k[this->index]);
//   if (check_feasibility({d_l1, 100.0}))
//   {
//       double cost_linear_100 = evaluate_cost({d_l1, 100.0}, rho);
//       if (cost_linear_100 < cost_best)
//       {
//           d_best = {d_l1, 100.0};
//           cost_best = cost_linear_100;
//       }
//   }

//   this->d = d_best;
//   this->cost = cost_best;

//   return *this;
// }