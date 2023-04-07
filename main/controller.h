#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
public:
  void set_controller(int LUMINAIRE);
  double get_control_signal(double r, double y, double h);
  double get_duty_cycle();

  void set_occupancy(int occupancy);
  int get_occupancy();
  void set_anti_windup(int anti_windup);
  int get_anti_windup();
  void set_feedback(int feedback);
  int get_feedback();

  double get_u();

  void set_k(double K);
  double get_k();
  void set_ti(double Ti);
  double get_ti();
  void set_b(double b);
  double get_b();
  void set_tt(double Tt);
  double get_tt();

  void set_modeOp(char subcmd, int i);

  /* For Stage 2 */
  void set_lower_bound_Occupied(double lower_bound_Occupied);
  double get_lower_bound_Occupied();
  void set_lower_bound_Unoccupied(double lower_bound_Unoccupied);
  double get_lower_bound_Unoccupied();
  void set_lower_bound();
  double get_lower_bound();
  void set_cost(double cost);
  double get_cost();

private:
  double _K, _Ti, _b, _Tt, _up, _ui, _u;
  int _occupancy, _anti_windup, _feedback;
  double _Kold, _bold;
  /* For Stage 2 */
  double _lower_bound_Occupied, _lower_bound_Unoccupied, _lower_bound, _cost;
};

#endif