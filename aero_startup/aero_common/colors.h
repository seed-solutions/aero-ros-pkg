#ifndef _AERO_COMMON_COLORS_
#define _AERO_COMMON_COLORS_

#include "aero_common/types.h"

namespace aero
{
  namespace colors
  {

    //////////////////////////////////////////////////
    inline aero::hsi rgb2hsi(aero::rgb _color)
    {
      aero::hsi hsi_color;

      // I
      float I_max = std::max({_color.r, _color.g, _color.b});
      hsi_color.i = I_max;

      // S
      float i_min = std::min({_color.r, _color.g, _color.b});
      if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
      else hsi_color.s = 0;

      // H
      if (static_cast<int>(I_max) == static_cast<int>(i_min))
      {
	hsi_color.h = 0;
      }
      else
      {
	if (_color.g > _color.b)
	{
	  if (_color.r > _color.g)
	    hsi_color.h = 60 * (_color.g - _color.b) / (I_max - i_min);
	  else
	    hsi_color.h = 60 * (2 + (_color.b - _color.r) / (I_max - i_min));
	}
	else
	{
	  if (_color.r > _color.b)
	    hsi_color.h = 360 - 60 * (_color.b - _color.g) / (I_max - i_min);
	  else
	    hsi_color.h = 60 * (4 + (_color.r - _color.g) / (I_max - i_min));
	}
	if (hsi_color.h <= 180) // 0 ~ 180 -> 0 ~ 127
	  hsi_color.h = static_cast<int>(hsi_color.h / 180.0 * 127);
	else // 180 ~ 360 -> -128 ~ 0
	  hsi_color.h = static_cast<int>((hsi_color.h - 360) / 180.0 * 128);
      }

      return hsi_color;
    };

    //////////////////////////////////////////////////
    inline aero::lab rgb2lab(aero::rgb _color)
    {
      aero::xyz adapt;
      adapt.x = 0.950467;
      adapt.y = 1.0;
      adapt.z = 1.088969;

      std::function<float(float)> g = [=](float _v)
      {
	_v = _v * 0.003922;
	if (_v <= 0.04045) return _v * 0.07739938;
	else return std::pow((_v + 0.055) / 1.055, 2.4);
      };

      aero::rgb rgb;
      rgb.r = g(_color.r);
      rgb.g = g(_color.g);
      rgb.b = g(_color.b);

      aero::xyz xyz;
      xyz.x = 0.412424 * rgb.r + 0.357579 * rgb.g + 0.180464 * rgb.b;
      xyz.y = 0.212656 * rgb.r + 0.715158 * rgb.g + 0.0721856 * rgb.b;
      xyz.z = 0.0193324 * rgb.r + 0.119193 * rgb.g + 0.950444 * rgb.b;

      std::function<float(float)> f = [=](float _v)
      {
	if (_v > 0.008856) return std::pow(_v, 0.333333);
	else return _v * 7.787 + 0.137931;
      };

      aero::lab lab;
      lab.l = 116 * f(xyz.y / adapt.y) - 16;
      lab.a = 500 * (f(xyz.x / adapt.x) - f(xyz.y / adapt.y));
      lab.b = 200 * (f(xyz.y / adapt.y) - f(xyz.z / adapt.z));

      return lab;
    };

    //////////////////////////////////////////////////
    inline float distance(aero::lab _color1, aero::lab _color2)
    {
      float L = (_color1.l + _color2.l) * 0.5;
      float dL = _color1.l - _color2.l;

      float C7 = std::pow(
	  (sqrt(_color1.a * _color1.a + _color1.b * _color1.b) +
	   sqrt(_color2.a * _color2.a + _color2.b * _color2.b)) * 0.5, 7);
      float sqC = sqrt(C7 / (C7 + 6103515625));
      float a1 = _color1.a * (1.0 + 0.5 * (1 - sqC));
      float a2 = _color2.a * (1.0 + 0.5 * (1 - sqC));
      float c1 = sqrt(a1 * a1 + _color1.b * _color1.b);
      float c2 = sqrt(a2 * a2 + _color2.b * _color2.b);
      float C = (c2 + c1) * 0.5;
      float dC = c2 - c1;

      if (a1 < 0.000001) a1 = 0.000001;
      if (a2 < 0.000001) a2 = 0.000001;
      float h1 = atan2(_color1.b, a1);
      if (h1 < 0) h1 += 2 * M_PI;
      float h2 = atan2(_color2.b, a2);
      if (h2 < 0) h2 += 2 * M_PI;
      float H = fabs(h2 - h1);
      if (H <= M_PI) H = (h1 + h2) * 0.5;
      else
      {
	H = h1 + h2;
	if (H < 2 * M_PI) H = H * 0.5 + M_PI;
	else H = H * 0.5 - M_PI;
      }
      float dh = h2 - h1;
      if (dh < -M_PI) dh += 2 * M_PI;
      else if (dh > M_PI) dh -= 2 * M_PI;
      float dH = 2 * sqrt(c1 * c2) * sin(dh * 0.5);

      float Sl = 1 + 0.015 * std::pow(L - 50, 2) / sqrt(20 + std::pow(L - 50, 2));
      float Sc = 1 + 0.045 * C;
      float Sh = 1 + 0.015 * C *
	(1 - 0.17 * cos(H - 0.5235988) + 0.24 * cos(2 * H) +
	 0.32 * cos(3 * H + 0.1047198) - 0.20 * cos(4 * H - 1.0995574));
      float Rt = -2 * sqC *
	  sin(1.0471976 * std::exp(-std::pow((H - 4.7996554) / 0.4363323, 2)));

      return sqrt(std::pow(dL / Sl, 2) + std::pow(dC / Sc, 2) +
		  std::pow(dH / Sh, 2) + Rt * dC * dH / (Sc * Sh));
    };

    //////////////////////////////////////////////////
    inline bool compare(aero::rgb _color, aero::hsi _hsi_min, aero::hsi _hsi_max)
    {
      aero::hsi hsi_color;

      float I_max = std::max({_color.r, _color.g, _color.b});
      hsi_color.i = I_max;
      if (hsi_color.i < _hsi_min.i || hsi_color.i > _hsi_max.i)
	return false;

      float i_min = std::min({_color.r, _color.g, _color.b});
      if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
      else hsi_color.s = 0;
      if (hsi_color.s < _hsi_min.s || hsi_color.s > _hsi_max.s)
	return false;

      if (static_cast<int>(I_max) == static_cast<int>(i_min))
      {
	hsi_color.h = 0;
      }
      else
      {
	if (_color.g > _color.b)
	{
	  if (_color.r > _color.g)
	    hsi_color.h = 60 * (_color.g - _color.b) / (I_max - i_min);
	  else
	    hsi_color.h = 60 * (2 + (_color.b - _color.r) / (I_max - i_min));
	}
	else
	{
	  if (_color.r > _color.b)
	    hsi_color.h = 360 - 60 * (_color.b - _color.g) / (I_max - i_min);
	  else
	    hsi_color.h = 60 * (4 + (_color.r - _color.g) / (I_max - i_min));
	}
	if (hsi_color.h <= 180)
	  hsi_color.h = static_cast<int>(hsi_color.h / 180.0 * 127);
	else
	  hsi_color.h = static_cast<int>((hsi_color.h - 360) / 180.0 * 128);
      }
      if (hsi_color.h < _hsi_min.h || hsi_color.h > _hsi_max.h)
	return false;

      return true;
    };

  }
}

#endif
