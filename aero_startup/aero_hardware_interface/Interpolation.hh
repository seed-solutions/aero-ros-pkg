#ifndef AERO_INTERPOLATION_H_
#define AERO_INTERPOLATION_H_

#include <vector>
#include <map>
#include <functional>
#include <memory>

namespace aero
{
  namespace interpolation
  {

    static const int i_constant = 0;

    static const int i_linear = 1;

    static const int i_bezier = 2;

    static const int i_slowin = 3;

    static const int i_slowout = 4;

    static const int i_sigmoid = 5;

    static const int i_cbezier = 6;

    class Interpolation
    {
    public: Interpolation(int id);

    public: ~Interpolation();

    public: bool is(const int id);

    public: std::function<float(float)> interpolate;

    public: std::function<void(std::pair<float, float>, int)> set_points;

    private: void init(const int n);

    private: void initConstant();
    private: float Constant(float t);
    private: void set_constant_p(std::pair<float, float> p, int at) {};

    private: void initLinear();
    private: float Linear(float t);
    private: void set_linear_p(std::pair<float, float> p, int at) {};

    private: void initBezier();
    private: float Bezier(float t);
    private: void set_bezier_p(std::pair<float, float> p, int at=0);

    private: void initSlowIn();
    private: float SlowIn(float t);
    private: void set_slowin_p(std::pair<float, float> p, int at=0);

    private: void initSlowOut();
    private: float SlowOut(float t);
    private: void set_slowout_p(std::pair<float, float> p, int at=0);

    private: void initSigmoid();
    private: float Sigmoid(float t);
    private: void set_sigmoid_p(std::pair<float, float> p, int at);

    private: void initCubicBezier();
    private: float CubicBezier(float t);
    private: void set_cubicbezier_p(std::pair<float, float> p, int at);

    private: int id_;

    private: std::vector<std::pair<float, float> > points_;
    };

    typedef std::shared_ptr<Interpolation> InterpolationPtr;
  }
}

#endif
