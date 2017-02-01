#include "Interpolation.hh"

using namespace aero;
using namespace interpolation;

Interpolation::Interpolation(int id)
{
  id_ = id;

  switch (id) {
  case i_constant:
    initConstant();
    interpolate = [&](float t){ return Constant(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_constant_p(p, at);
    };
    break;
  case i_linear:
    initLinear();
    interpolate = [&](float t){ return Linear(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_linear_p(p, at);
    };
    break;
  case i_bezier:
    initBezier();
    interpolate = [&](float t){ return Bezier(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_bezier_p(p, at);
    };
    break;
  case i_slowout:
    initSlowOut();
    interpolate = [&](float t){ return SlowOut(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_slowout_p(p, at);
    };
    break;
  case i_slowin:
    initSlowIn();
    interpolate = [&](float t){ return SlowIn(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_slowin_p(p, at);
    };
    break;
  case i_sigmoid:
    initSigmoid();
    interpolate = [&](float t){ return Sigmoid(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_sigmoid_p(p, at);
    };
    break;
  case i_cbezier:
    initCubicBezier();
    interpolate = [&](float t){ return CubicBezier(t); };
    set_points = [&](std::pair<float, float> p, int at) {
      set_cubicbezier_p(p, at);
    };
    break;
  }
}

Interpolation::~Interpolation() {
}

bool Interpolation::is(const int id)
{
  return (id_ == id);
}

void Interpolation::init(const int n)
{
  points_.clear();
  points_.resize(n);
  points_.at(0) = {0.0, 0.0};
  points_.at(n-1) = {1.0, 1.0};
}

void Interpolation::initConstant()
{
  points_.clear();
  points_.resize(2);
  points_.at(0) = {0.0, 0.0};
  points_.at(1) = {1.0, 0.0};
}

void Interpolation::initLinear()
{
  init(2);
}

void Interpolation::initBezier()
{
  init(3);
  points_.at(1) = {0.0, 1.0};
}

void Interpolation::initSlowOut()
{
  init(4);
  std::pair<float, float> p1(0.5, 0.8); // p1.first < p1.second  
  points_.at(1) = p1;
  points_.at(2) = {p1.first/p1.second, 1.0};
}

void Interpolation::initSlowIn()
{
  init(4);
  std::pair<float, float> p2(0.5, 0.2); // p2.x > p2.y
  points_.at(1) = {(p2.first-p2.second)/(1-p2.second), 0.0};
  points_.at(2) = p2;
}

void Interpolation::initSigmoid()
{
  init(6);
  std::pair<float, float> p2(0.3, 0.2); // p2.x < p3.x, p2.y < p3.y
  std::pair<float, float> p3(0.7, 0.8);
  points_.at(1) =
    {(p2.first*p3.second-p3.first*p2.second)/(p3.second-p2.second), 0.0};
  points_.at(2) = p2;
  points_.at(3) = p3;
  points_.at(4) = 
    {(p3.first*(1-p2.second)-p2.first*(1-p3.second))/(p3.second-p2.second),
     1.0};
}

void Interpolation::initCubicBezier()
{
  init(4);
  points_.at(1) = {0.0, 0.5};
  points_.at(2) = {1.0, 0.5};
}

float Interpolation::Constant(float t)
{
  return 0.0;
}

float Interpolation::Linear(float t)
{
  return t;
}

float Interpolation::Bezier(float t)
{
  return 2*t*(1-t)*points_.at(1).second + t*t;
}

float Interpolation::SlowOut(float t)
{
  float tA = t / points_.at(1).first;
  float tB = (t - points_.at(1).first) / (1 - points_.at(1).first);
  float sB = 1 - tB;
  return t <= points_.at(1).first ?
    tA*points_.at(1).second :
    sB*sB*points_.at(1).second + 2*sB*tB*points_.at(2).second + tB*tB;
}

float Interpolation::SlowIn(float t)
{
  float tA = t / points_.at(2).first;
  float tB = (t - points_.at(2).first) / (1 - points_.at(2).first);
  return t <= points_.at(2).first ?
    2*tA*(1-tA)*points_.at(1).second + tA*tA*points_.at(2).second :
    (1 - tB)*points_.at(2).second + tB;
}

float Interpolation::Sigmoid(float t)
{
  float tA = t / points_.at(2).first;
  float tB =
    (t - points_.at(2).first) / (points_.at(3).first - points_.at(2).first);
  float tC = (t - points_.at(3).first) / (1 - points_.at(3).first);
  float sC = 1 - tC;
  return t <= points_.at(2).first ?
    2*tA*(1-tA)*points_.at(1).second + tA*tA*points_.at(2).second :
    (t <= points_.at(3).first ?
     (1 - tB)*points_.at(2).second + tB*points_.at(3).second :
     sC*sC*points_.at(3).second + 2*sC*tC*points_.at(4).second + tC*tC);
}

float Interpolation::CubicBezier(float t)
{
  float s = 1 - t;
  return 3*s*s*t*points_.at(1).second + 3*t*t*s*points_.at(2).second
    + t*t*t;
}

void Interpolation::set_bezier_p(std::pair<float, float> p, int id)
{
  points_.at(1) = p; 
}

void Interpolation::set_slowout_p(std::pair<float, float> p, int id)
{
  points_.at(1) = p;
  if (points_.at(1).first > points_.at(1).second)
    points_.at(1).first = points_.at(1).second;
  points_.at(2) = {points_.at(1).first/points_.at(1).second, 1.0};
}

void Interpolation::set_slowin_p(std::pair<float, float> p, int id)
{
  points_.at(2) = p;
  if (points_.at(2).first < points_.at(2).second)
    points_.at(2).first = points_.at(2).second;
  points_.at(1) =
    {(points_.at(2).first-points_.at(2).second)/(1-points_.at(2).second),
     0.0};
}

void Interpolation::set_sigmoid_p(std::pair<float, float> p, int id)
{
  if (id == 2) {
    points_.at(2) = p;
    if (points_.at(2).first > points_.at(3).first)
      points_.at(2).first = points_.at(3).first;
    if (points_.at(2).second > points_.at(3).second)
      points_.at(2).second = points_.at(3).second;
  } else if (id == 3) {
    points_.at(3) = p;
    if (points_.at(3).first < points_.at(2).first)
      points_.at(3).first = points_.at(2).first;
    if (points_.at(3).second < points_.at(2).second)
      points_.at(3).second = points_.at(2).second;
  } else {
    return;
  }
  
  points_.at(1) =
    {(points_.at(2).first*points_.at(3).second
      - points_.at(3).first*points_.at(2).second)
     / (points_.at(3).second - points_.at(2).second),
     0.0};
  points_.at(4) =
    {(points_.at(3).first * (1-points_.at(2).second)
      - points_.at(2).first * (1-points_.at(3).second))
     / (points_.at(3).second-points_.at(2).second),
     1.0};
}

void Interpolation::set_cubicbezier_p(std::pair<float, float> p, int id)
{
  if (id == 1) {
    points_.at(1) = p;
    if (points_.at(1).first > points_.at(2).first)
      points_.at(1).first = points_.at(2).first;
  } else if (id == 2) {
    points_.at(2) = p;
    if (points_.at(2).first < points_.at(1).first)
      points_.at(2).first = points_.at(1).first;
  }
}
