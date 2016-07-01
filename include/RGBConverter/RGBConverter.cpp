/*
 * RGBConverter.h - Arduino library for converting between RGB, HSV and HSL
 *
 * Ported from the Javascript at http://mjijackson.com/2008/02/rgb-to-hsl-and-rgb-to-hsv-color-model-conversion-algorithms-in-javascript
 * The hard work was Michael's, all the bugs are mine.
 *
 * Robert Atkins, December 2010 (ratkins_at_fastmail_dot_fm).
 *
 * https://github.com/ratkins/RGBConverter
 *
 * Modifications by Aleksandar Vladimirov Atanasov, September 2015
 *  - all functions are now not bound to Arduino's header
 *  - added conversion for single and full RGB color values from/to double to/from integer
 *  - array arguments have been split into triplets of pointers (one pointer per value)
 *  - all functions are now static so there is no need for creating the RGBConverter object (constructor of RGBConverter has also been moved to private)
 *  - added alternative display of HSL using degrees and percentages
 *  - reformatted documentation
 */
#include "RGBConverter.h"
#include <cmath>

// Unit conversions
void RGBConverter::rgbIntToDouble_single(unsigned int x, double* _x) {
  *_x = x/255.;
}

void RGBConverter::rgbIntToDouble(unsigned int r, unsigned int g, unsigned int b, double* _r, double* _g, double* _b) {
  rgbIntToDouble_single(r, _r);
  rgbIntToDouble_single(g, _g);
  rgbIntToDouble_single(b, _b);
}

void RGBConverter::rgbDoubleToInt_single(double x, unsigned int* _x) {
  *_x = floor(x == 1.0 ? 255 : x * 256.0);
}

void RGBConverter::rgbDoubleToInt(double r, double g, double b, unsigned int* _r, unsigned int* _g, unsigned int* _b) {
  rgbDoubleToInt_single(r, _r);
  rgbDoubleToInt_single(g, _g);
  rgbDoubleToInt_single(b, _b);
}

void RGBConverter::hslIntervalZeroOneToDegAndPercentage(double h, double s, double l, double* _h, double* _s, double* _l) {
  *_h = h*360.;
  *_s = s*100.;
  *_l = l*100.;
}

void RGBConverter::hslDegAndPercentageToIntervalZeroOne(double h, double s, double l, double* _h, double* _s, double* _l) {
  *_h = h/360.;
  *_s = s/100.;
  *_l = l/100.;
}

// Color value conversions
void RGBConverter::rgbToHsl(double r, double g, double b, double* h, double* s, double* l) {
  double max = threeway_max(r, g, b);
  double min = threeway_min(r, g, b);
  *l = (max + min) / 2;

  if (max == min) {
   * h =* s = 0.; // achromatic
  } else {
    double d = max - min;
   *s = *l > .5 ? d / (2. - max - min) : d / (max + min);
    if (max == r) *h = (g - b) / d + (g < b ? 6. : 0.);
    else if (max == g) *h = (b - r) / d + 2.;
    else if (max == b) *h = (r - g) / d + 4.;

   *h /= 6;
  }
}

void RGBConverter::hslToRgb(double h, double s, double l, double* r, double* g, double* b) {
  if (s == 0.) {
   *r =*g =*b = l; // achromatic
  } else {
    double q = l < .5 ? l * (1 + s) : l + s - l * s;
    double p = 2. * l - q;
   *r = RGBConverter::hueToRgb(p, q, h + 1/3.);
   *g = RGBConverter::hueToRgb(p, q, h);
   *b = RGBConverter::hueToRgb(p, q, h - 1/3.);
  }
}

void RGBConverter::rgbToHsv(double r, double g, double b, double* h, double* s, double* v) {
  double max = threeway_max(r, g, b), min = threeway_min(r, g, b);
  *v = max;

  double d = max - min;
  *s = max == 0. ? 0. : d / max;

  if (max == min) {
   *h = 0.; // achromatic
  } else {
    if (max == r)* h = (g - b) / d + (g < b ? 6. : 0.);
    else if (max == g) *h = (b - r) / d + 2.;
    else if (max == b) *h = (r - g) / d + 4.;

   *h /= 6.;
  }
}

void RGBConverter::hsvToRgb(float h, float s, float v, float* r, float* g, float* b) {
  unsigned int i = static_cast<unsigned int>(h*  6.);
  double f = h * 6. - i;
  double p = v * (1. - s);
  double q = v * (1. - f * s);
  double t = v * (1. - (1. - f) * s);

  switch(i % 6){
    case 0: *r = v, *g = t, *b = p; break;
    case 1: *r = q, *g = v, *b = p; break;
    case 2: *r = p, *g = v, *b = t; break;
    case 3: *r = p, *g = q, *b = v; break;
    case 4: *r = t, *g = p, *b = v; break;
    case 5: *r = v, *g = p, *b = q; break;
  }
}

void RGBConverter::hsvToRgb(double h, double s, double v, double* r, double* g, double* b) {
  unsigned int i = static_cast<unsigned int>(h*  6.);
  double f = h * 6. - i;
  double p = v * (1. - s);
  double q = v * (1. - f * s);
  double t = v * (1. - (1. - f) * s);

  switch(i % 6){
    case 0: *r = v, *g = t, *b = p; break;
    case 1: *r = q, *g = v, *b = p; break;
    case 2: *r = p, *g = v, *b = t; break;
    case 3: *r = p, *g = q, *b = v; break;
    case 4: *r = t, *g = p, *b = v; break;
    case 5: *r = v, *g = p, *b = q; break;
  }
}

double RGBConverter::threeway_max(double a, double b, double c) {
  return fmax(a, fmax(b, c));
}

double RGBConverter::threeway_min(double a, double b, double c) {
  return fmin(a, fmin(b, c));
}

double RGBConverter::hueToRgb(double p, double q, double t) {
  if(t < 0.) t += 1;
  if(t > 1.) t -= 1;
  if(t < 1/6.) return p + (q - p) * 6. * t;
  if(t < 1/2.) return q;
  if(t < 2/3.) return p + (q - p) * (2/3. - t) * 6.;
  return p;
}

void RGBConverter::rgbToCmyk(double r, double g, double b, double* c, double* m, double* y, double* k) {
 *k = 1. - threeway_max(r, g, b);
 *c = (1. - r - *k) / (1. - *k);
 *m = (1. - g - *k) / (1. - *k);
 *y = (1. - b - *k) / (1. - *k);
}

void RGBConverter::cmykToRgb(double c, double m, double y, double k, double* r, double* g, double* b) {
 *r = (1. - c) / (1. - k);
 *g = (1. - m) / (1. - k);
 *b = (1. - y) / (1. - k);
}

void RGBConverter::rgbToYiq(double r, double g, double b, double* y, double* i, double* q) {
 *y = .299 * r + .587 * g + .114 * b;
 *i = .569 * r - .275 * g - .322 * b;
 *q = .211 * r - .523 * g + .312 * b;
}

void RGBConverter::yiqToRgb(double y, double i, double q, double* r, double* g, double* b) {
 *r = y + .956 * i + .621 * q;
 *g = y - .272 * i - .647 * q;
 *b = y - 1.106 * i + 1.703 * q;
}

void RGBConverter::hslToHsv(double h, double s, double l, double* _h, double* _s, double* v) {
  double r, g, b;
  hslToRgb(h, s, l, &r, &g, &b);
  rgbToHsv(r, g, b, _h, _s, v);
}

void RGBConverter::hslToCmyk(double h, double s, double l, double* c, double* m, double* y, double* k) {
  double r, g, b;
  hslToRgb(h, s, l, &r, &g, &b);
  rgbToCmyk(r, g, b, c, m, y, k);
}

void RGBConverter::hslToYiq(double h, double s, double l, double* y, double* i, double* q) {
  double r, g, b;
  hslToRgb(h, s, l, &r, &g, &b);
  rgbToYiq(r, g, b, y, i, q);
}

void RGBConverter::hsvToHsl(double h, double s, double v, double* _h, double* _s, double* l) {
  double r, g, b;
  hsvToRgb(h, s, v, &r, &g, &b);
  rgbToHsl(r, g, b, _h, _s, l);
}

void RGBConverter::hsvToCmyk(double h, double s, double v, double* c, double* m, double* y, double* k) {
  double r, g, b;
  hsvToRgb(h, s, v, &r, &g, &b);
  rgbToCmyk(r, g, b, c, m, y, k);
}

void RGBConverter::hsvToYiq(double h, double s, double v, double *y, double *i, double *q) {
  double r, g, b;
  hsvToRgb(h, s, v, &r, &g, &b);
  rgbToYiq(r, g, b, y, i, q);
}
