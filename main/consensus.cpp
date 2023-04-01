#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

// Assuming Node is a struct or class with appropriate properties
struct Node
{
    double d_av, y, c, cost;
    std::vector<double> k, o, l;
    std::vector<double> d;
    int n, m, index;
};

bool check_feasibility(const Node &node, const std::vector<double> &d)
{
    const double tol = 0.001; // tolerance for rounding errors

    if (d[node.index] < -tol)
    {
        return false;
    }

    if (d[node.index] > 100 + tol)
    {
        return false;
    }

    double d_dot_k = 0;
    for (size_t i = 0; i < d.size(); ++i)
    {
        d_dot_k += d[i] * node.k[i];
    }

    if (d_dot_k < node.L[node.index] - node.o[node.index] - tol)
    {
        return false;
    }

    return true;
}

double evaluate_cost(const Node& node, const std::vector<double>& d, double rho) {
    double cost = 0.0;

    for (size_t i = 0; i < d.size(); ++i) {
        cost += node.c * d[i] + node.y * (d[i] - node.d_av);
    }

    double norm_squared = 0.0;
    for (size_t i = 0; i < d.size(); ++i) {
        norm_squared += std::pow(d[i] - node.d_av, 2);
    }

    cost += rho / 2 * norm_squared;

    return cost;
}

Node consensus_iterate(const Node &node, double rho, std::vector<double> &d, double &cost)
{
    std::vector<double> d_best(2, -1);
    double cost_best = 1000000; // large number
    double z = rho * node.d_av - node.y - node.c;

    // unconstrained minimum
    double d_u = (1 / rho) * z;
    if (check_feasibility(node, {d_u}))
    {
        double cost_unconstrained = evaluate_cost(node, {d_u}, rho);
        if (cost_unconstrained < cost_best)
        {
            d_best = {d_u};
            cost_best = cost_unconstrained;
        }
    }

    // compute minimum constrained to linear boundary
    double d_bl = (1 / rho) * z - node.k[node.index] / node.n * (node.o[node.index] - node.l[node.index] + (1 / rho) * z * node.k[node.index]);
    if (check_feasibility(node, {d_bl}))
    {
        double cost_boundary_linear = evaluate_cost(node, {d_bl}, rho);
        if (cost_boundary_linear < cost_best)
        {
            d_best = {d_bl};
            cost_best = cost_boundary_linear;
        }
    }

    // compute minimum constrained to 0 boundary
    double d_b0 = (1 / rho) * z;
    if (check_feasibility(node, {d_b0, 0.0}))
    {
        double cost_boundary_0 = evaluate_cost(node, {d_b0, 0.0}, rho);
        if (cost_boundary_0 < cost_best)
        {
            d_best = {d_b0, 0.0};
            cost_best = cost_boundary_0;
        }
    }

    // compute minimum constrained to 100 boundary
    double d_b1 = (1 / rho) * z;
    if (check_feasibility(node, {d_b1, 100.0}))
    {
        double cost_boundary_100 = evaluate_cost(node, {d_b1, 100.0}, rho);
        if (cost_boundary_100 < cost_best)
        {
            d_best = {d_b1, 100.0};
            cost_best = cost_boundary_100;
        }
    }

    // compute minimum constrained to linear and 0 boundary
    double d_l0 = (1 / rho) * z - (1 / node.m) * node.k[node.index] * (node.o[node.index] - node.L[node.index]) + (1 / rho / node.m) * node.k[node.index] * (node.k[node.index] * z - z * node.k[node.index]);
    if (check_feasibility(node, {d_l0, 0.0}))
    {
        double cost_linear_0 = evaluate_cost(node, {d_l0, 0.0}, rho);
        if (cost_linear_0 < cost_best)
        {
            d_best = {d_l0, 0.0};
            cost_best = cost_linear_0;
        }
    }

    // compute minimum constrained to linear and 100 boundary
    double d_l1 = (1 / rho) * z - (1 / node.m) * node.k[node.index] * (node.o[node.index] - node.L[node.index] + 100 * node.k[node.index]) + (1 / rho / node.m) * node.k[node.index] * (node.k[node.index] * z - z * node.k[node.index]);
    if (check_feasibility(node, {d_l1, 100.0}))
    {
        double cost_linear_100 = evaluate_cost(node, {d_l1, 100.0}, rho);
        if (cost_linear_100 < cost_best)
        {
            d_best = {d_l1, 100.0};
            cost_best = cost_linear_100;
        }
    }

    node.d = d_best;
    node.cost = cost_best;

    return node
}