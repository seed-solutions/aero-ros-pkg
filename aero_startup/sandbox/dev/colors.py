#!/usr/bin/env python

from math import *

def g(_v):
    _v = _v * 0.003922
    if _v <= 0.04045:
        return _v * 0.07739938
    else:
        return pow((_v + 0.055) / 1.055, 2.4)

def f(_v):
    if _v > 0.008856:
        return pow(_v, 0.333333)
    else:
        return _v * 7.787 + 0.137931

def rgb2lab(_r, _g, _b):
    adapt = {'x' : 0.950467, 'y' : 1.0, 'z' : 1.088969}
    
    rgb = {'r' : g(_r), 'g' : g(_g), 'b' : g(_b)}
    
    xyz = {
        'x' : 0.412424 * rgb['r'] + 0.357579 * rgb['g'] + 0.180464 * rgb['b'],
        'y' : 0.212656 * rgb['r'] + 0.715158 * rgb['g'] + 0.0721856 * rgb['b'],
        'z' : 0.0193324 * rgb['r'] + 0.119193 * rgb['g'] + 0.950444 * rgb['b']}
    
    lab = {
        'l' : 116 * f(xyz['y'] / adapt['y']) - 16,
        'a' : 500 * (f(xyz['x'] / adapt['x']) - f(xyz['y'] / adapt['y'])),
        'b' : 200 * (f(xyz['y'] / adapt['y']) - f(xyz['z'] / adapt['z']))}
    
    return lab

def distance(_color1, _color2):
    L = (_color1['l'] + _color2['l']) * 0.5
    dL = _color1['l'] - _color2['l']
    
    C7 = pow(
        (sqrt(_color1['a'] * _color1['a'] + _color1['b'] * _color1['b']) +
         sqrt(_color2['a'] * _color2['a'] + _color2['b'] * _color2['b'])) * 0.5,
        7)
    sqC = sqrt(C7 / (C7 + 6103515625))
    a1 = _color1['a'] + _color1['a'] * 0.5 * (1 - sqC)
    a2 = _color2['a'] + _color2['a'] * 0.5 * (1 - sqC)
    c1 = sqrt(a1 * a1 + _color1['b'] * _color1['b'])
    c2 = sqrt(a2 * a2 + _color2['b'] * _color2['b'])
    C = (c2 + c1) * 0.5
    dC = c2 - c1
    
    if fabs(a1) < 0.000001:
        a1 = 0.000001
    if fbas(a2) < 0.000001:
        a2 = 0.000001
    h1 = atan2(_color1['b'], a1)
    if h1 < 0:
        h1 += 2 * pi
    h2 = atan2(_color2['b'], a2)
    if h2 < 0:
        h2 += 2 * pi
    H = fabs(h2 - h1)
    if H <= pi:
        H = (h1 + h2) * 0.5
    else:
        H = h1 + h2
        if H < 2 * pi:
            H = H * 0.5 + pi
        else:
            H = H * 0.5 - pi
    dh = h2 - h1
    if dh < -pi:
        dh += 2 * pi
    elif dh > pi:
        dh -= 2 * pi
    dH = 2 * sqrt(c1 * c2) * sin(dh * 0.5)
    
    Sl = 1 + 0.015 * pow(L - 50, 2) / sqrt(20 + pow(L - 50, 2))
    Sc = 1 + 0.045 * C
    Sh = 1 + 0.015 * C * (1 - 0.17 * cos(H - 0.5235988) +
                          0.24 * cos(2 * H) +
                          0.32 * cos(3 * H + 0.1047198) -
                          0.20 * cos(4 * H - 1.0995574))
    Rt = -2 * sqC * sin(1.0471976 * exp(-pow((H - 4.7996554) / 0.4363323, 2)))
    
    return sqrt(pow(dL / Sl, 2) + pow(dC / Sc, 2) +
                pow(dH / Sh, 2) + Rt * dC * dH / (Sc * Sh))
