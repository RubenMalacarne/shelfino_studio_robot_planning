#include "planning_pkg/utils/dubins.hpp"

/**
 * @brief Construct a new Dubins:: Dubins object
 * 
 */
Dubins::Dubins()
{
  function_vector = {
      &Dubins::d_lsl,
      &Dubins::d_rsr,
      &Dubins::d_lsr,
      &Dubins::d_rsl,
      &Dubins::d_rlr,
      &Dubins::d_lrl};
}

Dubins::~Dubins() {}

/**
 * @brief modulos operation for angles
 * 
 * @param angle angle to mod
 * 
 * @return double modulos angle
 */
double Dubins::mod2pi(double angle)
{
  double result = std::fmod(angle, 2 * M_PI);
  if (result < 0)
  {
    result += 2 * M_PI;
  }
  return result;
}

/**
 * @brief scale the input to the standard dubins problem
 */
std::vector<double> Dubins::scale_to_standard(double x0, double y0, double th0, double xf, double yf, double thf, double kmax)
{
  double dx = xf - x0;
  double dy = yf - y0;
  double phi = atan2(dy, dx);
  double lam = hypot(dx, dy) / 2;
  std::vector<double> result;
  result.push_back(this->mod2pi(th0 - phi)); // scaled th0
  result.push_back(this->mod2pi(thf - phi)); // scaled thf
  result.push_back(kmax * lam);              // scaled kmax
  result.push_back(lam);                     // scaled factor

  return result;
}

/**
 * @brief scale the output from the standard dubins problem
 */
std::vector<double> Dubins::scale_from_standard(double lam, double sc_s1, double sc_s2, double sc_s3)
{
  std::vector<double> result;
  result.push_back(lam * sc_s1); // s1
  result.push_back(lam * sc_s2); // s2
  result.push_back(lam * sc_s3); // s3
  return result;
}

/**
 * @brief sinc function
 */
double Dubins::sinc(double x)
{
  if (abs(x) < 0.002)
  {
    return 1 - pow(x, 1 / 3) * (1 - pow(x, 1 / 10));
  }
  else
  {
    return sin(x) / x;
  }
}

// DUBINS PRIMITIVES 

std::vector<double> Dubins::d_lsl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_thf) - cos(sc_th0);
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double sc_s1 = radious * this->mod2pi(temp1 - sc_th0);
  double temp2 = 2 + 4 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp2);
  double sc_s3 = radious * this->mod2pi(sc_thf - temp1);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rsr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(c, s);
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1);
  double temp2 = 2 + 4 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp2);
  double sc_s3 = radious * this->mod2pi(temp1 - sc_thf);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_lsr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) + cos(sc_thf);
  double s = 2 * sc_kmax + sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(-c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
  if (temp3 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp3);
  double temp2 = -atan2(-2, sc_s2 * sc_kmax);
  double sc_s1 = radious * this->mod2pi(temp1 + temp2 - sc_th0);
  double sc_s3 = radious * this->mod2pi(temp1 + temp2 - sc_thf);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rsl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) + cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp3 = 4 * pow(sc_kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
  if (temp3 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * sqrt(temp3);
  double temp2 = atan2(2, sc_s2 * sc_kmax);
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1 + temp2);
  double sc_s3 = radious * this->mod2pi(sc_thf - temp1 + temp2);
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_rlr(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_th0) - cos(sc_thf);
  double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp2 = 0.125 * (6 - 4 * pow(sc_kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * this->mod2pi(2 * M_PI - acos(temp2));
  double sc_s1 = radious * this->mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_kmax);
  double sc_s3 = radious * this->mod2pi(sc_th0 - sc_thf + sc_kmax * (sc_s2 - sc_s1));
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::d_lrl(double sc_th0, double sc_thf, double sc_kmax)
{
  double radious = 1 / sc_kmax;
  double c = cos(sc_thf) - cos(sc_th0);
  double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(c, s);
  double temp2 = 0.125 * (6 - 4 * pow(sc_kmax, 2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
  if (temp2 < 0)
  {
    return {0, 0, 0};
  }
  double sc_s2 = radious * this->mod2pi(2 * M_PI - acos(temp2));
  double sc_s1 = radious * this->mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_kmax);
  double sc_s3 = radious * this->mod2pi(sc_thf - sc_th0 + sc_kmax * (sc_s2 - sc_s1));
  return {sc_s1, sc_s2, sc_s3};
}

std::vector<double> Dubins::call_function(int index, double sc_th0, double sc_thf, double sc_kmax)
{
  return (this->*function_vector[index])(sc_th0, sc_thf, sc_kmax);
}


/**
 * @brief build a dubins curve from the scaled quantities
 */
struct dubins_curve Dubins::build_dubins(double x0, double y0, double th0, double s1, double s2, double s3, double k1, double k2, double k3)
{
  struct dubins_arc a1 = this->build_dubins_arc(x0, y0, th0, k1, s1);
  struct dubins_arc a2 = this->build_dubins_arc(a1.xf, a1.yf, a1.thf, k2, s2);
  struct dubins_arc a3 = this->build_dubins_arc(a2.xf, a2.yf, a2.thf, k3, s3);

  struct dubins_curve result = {a1, a2, a3, a1.L + a2.L + a3.L};

  return result;
}

/**
 * @brief builds a dubins arc
 */
struct dubins_arc Dubins::build_dubins_arc(double x0, double y0, double th0, double k, double L)
{
  std::vector<double> end = this->circline(L, x0, y0, th0, k);

  struct dubins_arc result = {x0, y0, th0, L, k, end.at(0), end.at(1), end.at(2)};

  return result;
}

std::vector<double> Dubins::circline(double L, double x0, double y0, double th0, double k)
{
  double x, y, th;
  x = x0 + L * this->sinc(k * L / 2) * cos(th0 + L * k / 2);
  y = y0 + L * this->sinc(k * L / 2) * sin(th0 + L * k / 2);
  th = this->mod2pi(th0 + L * k);

  return {x, y, th};
}

/**
 * @brief found the shortest dubins path between two points
 */
struct dubins_curve Dubins::dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double kmax, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  std::vector<double> scaled_quantities = this->scale_to_standard(x0, y0, th0, xf, yf, thf, kmax);
  int pidx = -1;
  std::vector<double> best_dubins;
  bool is_valid = false;
  double L = 999;
  double Lcur = 999;
  std::vector<int> ksign = {1, 0, 1, -1, 0, -1, 1, 0, -1, -1, 0, 1, -1, 1, -1, 1, -1, 1};

  struct dubins_curve result;

  for (int i = 0; i < 6; i++)
  {
    std::vector<double> result_d = this->call_function(i, scaled_quantities.at(0), scaled_quantities.at(1), scaled_quantities.at(2));
    Lcur = result_d.at(0) + result_d.at(1) + result_d.at(2);

    struct dubins_curve tmp_curve;

    if (!(result_d.at(0) == 0 && result_d.at(1) == 0 && result_d.at(2) == 0))
    {
      std::vector<double> aux_1 = this->scale_from_standard(scaled_quantities.at(3), result_d.at(0), result_d.at(1), result_d.at(2));
      tmp_curve = this->build_dubins(x0, y0, th0, aux_1.at(0), aux_1.at(1), aux_1.at(2), ksign.at(3 * i) * kmax, ksign.at(3 * i + 1) * kmax, ksign.at(3 * i + 2) * kmax);

      if (Lcur < L && this->valid_curve(tmp_curve, map))
      {
        is_valid = true;
        best_dubins = result_d;
        L = Lcur;
        pidx = i;
      }
    }
  }
  if (pidx >= 0)
  {

    std::vector<double> s = this->scale_from_standard(scaled_quantities.at(3), best_dubins.at(0), best_dubins.at(1), best_dubins.at(2));

    result = this->build_dubins(x0, y0, th0, s.at(0), s.at(1), s.at(2), ksign.at(3 * pidx) * kmax, ksign.at(3 * pidx + 1) * kmax, ksign.at(3 * pidx + 2) * kmax);
  }

  if (!is_valid)
  {
    throw std::runtime_error("No valid Dubins curve found.");
  }

  return result;
}

/**
 * @brief found the shortest multipoint dubins path 
 */
std::vector<struct dubins_curve> Dubins::dubins_multi_point(double x0, double y0, double th0, double xf, double yf, double thf, std::vector<std::vector<double>> points, double kmax, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  std::vector<struct dubins_curve> curves;
  std::vector<double> known_point = {xf, yf, thf};

  for (auto it = points.rbegin(); it != points.rend(); ++it)
  {
    const auto &p = *it;
    double L = 999;
    double Lcur;
    struct dubins_curve best_curve;

    // find best
    for (uint deg = 0; deg < 360; deg++)
    {
      double rad = deg * M_PI / 180;
      auto aux_curve = this->dubins_shortest_path(p.at(0), p.at(1), rad, known_point.at(0), known_point.at(1), known_point.at(2), kmax, map);
      Lcur = aux_curve.L;
      if (Lcur < L)
      {
        // std::cout << "maus" << std::endl;
        best_curve = aux_curve;
        L = Lcur;
      }
    }
    known_point.at(0) = best_curve.a1.x0;
    known_point.at(1) = best_curve.a1.y0;
    known_point.at(2) = best_curve.a1.th0;


    curves.insert(curves.begin(), best_curve);
  }

  // insert last
  curves.insert(curves.begin(), this->dubins_shortest_path(x0, y0, th0, known_point.at(0), known_point.at(1), known_point.at(2), kmax, map));

  return curves;
}

/**
 * @brief check if a curve is valid
 */
bool Dubins::valid_curve(struct dubins_curve curve, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  // transform the curve in a series of points and check if the point is inside the map or not
  auto path = this->sample_dubins_curve(curve, 0.05);
  for (auto &point : path)
  {
    if (!boost::geometry::within(point, map))
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief sample points along a dubins arc
 */
std::vector<point_t> Dubins::sample_dubins_arc(const dubins_arc &arc, double step_size)
{
  std::vector<point_t> points;

  // Number of steps along the arc
  int num_steps = static_cast<int>(std::ceil(arc.L / step_size));
  for (int i = 0; i <= num_steps; ++i)
  {
    point_t p;
    double s = i * step_size; // Distance along the arc
    if (s > arc.L)
      s = arc.L;

    double x, y, theta;
    if (arc.k == 0) // Straight line
    {
      x = arc.x0 + s * std::cos(arc.th0);
      y = arc.y0 + s * std::sin(arc.th0);
    }
    else // Circular arc
    {
      double r = 1.0 / std::abs(arc.k); // Radius of the arc
      double cx = arc.x0 - r * std::sin(arc.th0); // Center of the circle
      double cy = arc.y0 + r * std::cos(arc.th0);

      double delta_theta = s * arc.k; // Change in angle along the arc
      theta = arc.th0 + delta_theta;
      x = cx + r * std::sin(theta);
      y = cy - r * std::cos(theta);
    }

    // Add the point to the list
    p.set<0>(x);
    p.set<1>(y);
    points.push_back(p);
  }

  return points;
}

/**
 * @brief sample points along a dubins curve
 */
std::vector<point_t> Dubins::sample_dubins_curve(const dubins_curve &curve, double step_size)
{
  std::vector<point_t> points;

  // Sample each arc and concatenate the points
  auto arc1_points = sample_dubins_arc(curve.a1, step_size);
  auto arc2_points = sample_dubins_arc(curve.a2, step_size);
  auto arc3_points = sample_dubins_arc(curve.a3, step_size);

  points.insert(points.end(), arc1_points.begin(), arc1_points.end());
  points.insert(points.end(), arc2_points.begin(), arc2_points.end());
  points.insert(points.end(), arc3_points.begin(), arc3_points.end());

  return points;
}