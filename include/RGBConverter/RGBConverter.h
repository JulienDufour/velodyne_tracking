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
 *  - added CMYK and YIQ support
 *  - added wrappers for converting between non-RGB color values (example: HSL to HSV)
 */
#ifndef RGBConverter_h
#define RGBConverter_h

// TODO Testing

class RGBConverter {

public:
    // Unit conversions
    /**
     *  @brief rgbIntToDouble_single Converts a single integer RGB color value component (red, green or blue) into its floating point representation
     *  @param x   RGB single component as integer                                   (input)
     *  @param _x  RGB single component as floating point                            (output)
     */
    static void rgbIntToDouble_single(unsigned int x, double* _x);
    /**
     *  @brief rgbIntToDouble Converts a RGB color value (red, green and blue) into its floating point representation
     *  @param r     RGB red as integer in the interval [0 .. 255]                   (input)
     *  @param g     RGB green as integer in the interval [0 .. 255]                 (input)
     *  @param b     RGB blue as integer in the interval [0 .. 255]                  (input)
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (output)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (output)
     */
    static void rgbIntToDouble(unsigned int r, unsigned int g, unsigned int b, double* _r, double* _g, double* _b);
    /**
     *  @brief rgbDoubleToInt_single Converts a single floating point RGB color value component (red, green or blue) into its integer representation
     *  @param x   RGB single component as floating point in the interval [0.0 .. 1.0]   (input)
     *  @param _x  RGB single component as integer in the interval [0 .. 255]            (output)
     */
    static void rgbDoubleToInt_single(double x, unsigned int* _x);
    /**
     *  @brief rgbDoubleToInt Converts a floating point RGB color value (red, green and blue) into its integer representation
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (input)
     *  @param r     RGB red as integer in the interval [0 .. 255]                   (output)
     *  @param g     RGB green as integer in the interval [0 .. 255]                 (output)
     *  @param b     RGB blue as integer in the interval [0 .. 255]                  (output)
     */
    static void rgbDoubleToInt(double r, double g, double b, unsigned int* _r, unsigned int* _g, unsigned int* _b);
    /**
     *  @brief hslIntervalZeroOneToDegAndPercentage Converts a HSL color value with each component in the interval [0.0 .. 1.0] to degrees (for hue) and percentages (for saturation and lightness)
     *  @param h     HSL hue as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param s     HSL saturation as floating point in the interval [0.0 .. 1.0]   (input)
     *  @param l     HSL lightness as floating point in the interval [0.0 .. 1.0]    (input)
     *  @param _h    HSL hue as floating point in the interval [0.0 .. 360.0deg]     (output)
     *  @param _s    HSL saturation as floating point in the interval [0.0 .. 100.0%](output)
     *  @param _l    HSL lightness as floating point in the interval [0.0 .. 100.0%] (output)
     */
    static void hslIntervalZeroOneToDegAndPercentage(double h, double s, double l, double* _h, double* _s, double* _l);   // Note: all numbers after the floating point remain intact and represent minutes+seconds; if minutes and seconds are needed to be displayed, this function can be extended to conver those numbers into the desired measurement units
    /**
     *  @brief hslDegAndPercentageToIntervalZeroOne Converts a HSL color value represented as degrees (for hue) and percentages (for saturation and lightness) to representation where each component in the interval [0.0 .. 1.0]
     *  @param h     HSL hue as floating point in the interval [0.0 .. 360.0deg]     (input)
     *  @param s     HSL saturation as floating point in the interval [0.0 .. 100.0%](input)
     *  @param l     HSL lightness as floating point in the interval [0.0 .. 100.0%] (input)
     *  @param _h    HSL hue as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param _s    HSL saturation as floating point in the interval [0.0 .. 1.0]   (output)
     *  @param _l    HSL lightness as floating point in the interval [0.0 .. 1.0]    (output)
     */
    static void hslDegAndPercentageToIntervalZeroOne(double h, double s, double l, double* _h, double* _s, double* _l);   // TODO

    // Color value conversions
    // RGB/HSV
    /**
     *  @brief rgbToHsl Converts a floating point RGB color value (red, green and blue) into its HSL (hue, saturation and lightness) representation (all components for both RGB and HSL are in the interval [0.0 .. 1.0]
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (input)
     *  @param h     HSL hue as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param s     HSL saturation as floating point in the interval [0.0 .. 1.0]   (output)
     *  @param l     HSL lightness as floating point in the interval [0.0 .. 1.0]    (output)
     */
    static void rgbToHsl(double r, double g, double b, double* h, double* s, double* l);
    /**
     *  @brief hslToRgb Converts a floating point RGB color value (red, green and blue) into its HSL (hue, saturation and lightness) representation (all components in the interval [0.0 .. 1.0]
     *  @param h     HSL hue as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param s     HSL saturation as floating point in the interval [0.0 .. 1.0]   (input)
     *  @param l     HSL lightness as floating point in the interval [0.0 .. 1.0]    (input)
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (output)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (output)
     */
    static void hslToRgb(double h, double s, double l, double* r, double* g, double* b);

    // RGB/HSV
    /**
     *  @brief rgbToHsv Converts a floating point RGB color value (red, green and blue) into its HSV (hue, saturation and value) representation (all components in the interval [0.0 .. 1.0]
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (input)
     *  @param h     HSV hue as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param s     HSV saturation as floating point in the interval [0.0 .. 1.0]   (output)
     *  @param v     HSV value as floating point in the interval [0.0 .. 1.0]        (output)
     */
    static void rgbToHsv(double r, double g, double b, double* h, double* s, double* v);
    /**
     *  @brief hsvToRgb
     *  @param h     HSV hue as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param s     HSV saturation as floating point in the interval [0.0 .. 1.0]   (input)
     *  @param l     HSV value as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param g     RGB green as floating point in the interval [0.0 .. 1.0]        (output)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (output)
     */
    static void hsvToRgb(float h, float s, float v, float* r, float* g, float* b);
    static void hsvToRgb(double h, double s, double v, double* r, double* g, double* b);

    // RGB/CMYK
    /**
     *  @brief rgbToCmyk Converts a floating point RGB color value (red, green and blue) into its CMYK (cyan, magenta, yellow and black) representation (all components in the interval [0.0 .. 1.0]
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (input)
     *  @param g     RGB greem as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (input)
     *  @param c     CMYK cyan as floating point in the interval [0.0 .. 1.0]        (output)
     *  @param m     CMYK magenta as floating point in the interval [0.0 .. 1.0]     (output)
     *  @param y     CMYK yellow as floating point in the interval [0.0 .. 1.0]      (output)
     *  @param k     CMYK black as floating point in the interval [0.0 .. 1.0]       (output)
     */
    static void rgbToCmyk(double r, double g, double b, double* c, double* m, double* y, double* k);
    /**
     *  @brief cmykToRgb
     *  @param c     CMYK cyan as floating point in the interval [0.0 .. 1.0]        (input)
     *  @param m     CMYK magenta as floating point in the interval [0.0 .. 1.0]     (input)
     *  @param y     CMYK yellow as floating point in the interval [0.0 .. 1.0]      (input)
     *  @param k     CMYK black as floating point in the interval [0.0 .. 1.0]       (input)
     *  @param r     RGB red as floating point in the interval [0.0 .. 1.0]          (output)
     *  @param g     RGB greem as floating point in the interval [0.0 .. 1.0]        (output)
     *  @param b     RGB blue as floating point in the interval [0.0 .. 1.0]         (output)
     */
    static void cmykToRgb(double c, double m, double y, double k, double* r, double* g, double* b);

    // RGB/YIQ
    /**
     *  @brief rgbToYiq
     *  @param r
     *  @param g
     *  @param b
     *  @param y
     *  @param i
     *  @param q
     */
    static void rgbToYiq(double r, double g, double b, double* y, double* i, double* q);
    /**
     *  @brief yiqToRgb
     *  @param y
     *  @param i
     *  @param q
     *  @param r
     *  @param g
     *  @param b
     */
    static void yiqToRgb(double y, double i, double q, double* r, double* g, double* b);

    // ...

    // Color value conversions - wrappers (example: HSL to HSV, YIQ to HSL etc.)
    // Input/output parameters are similar to the ones mentioned above
    // HSL to ...
    static void hslToHsv(double h, double s, double l, double* _h, double* _s, double* v);
    static void hslToCmyk(double h, double s, double l, double* c, double* m, double* y, double* k);
    static void hslToYiq(double h, double s, double l, double* y, double* i, double* q);

    // HSV to ...
    static void hsvToHsl(double h, double s, double v, double* _h, double* _s, double* l);
    static void hsvToCmyk(double h, double s, double v, double* c, double* m, double* y, double* k);
    static void hsvToYiq(double h, double s, double v, double* y, double* i, double* q);

    // CMYK to ...
    static void cmykToHsl(double c, double m, double y, double k, double* h, double* s, double* l);
    static void cmykToHsv(double c, double m, double y, double k, double* h, double* s, double* v);
    static void cmykToYiq(double c, double m, double y, double k, double* _y, double* i, double* q);

    // YIQ to ...
    static void yiqToHsl(double y, double i, double q, double* h, double* s, double* l);
    static void yiqToHsv(double y, double i, double q, double* h, double* s, double* v);
    static void yiqToCmyk(double y, double i, double q, double* c, double* m, double* _y, double* k);

private:
    RGBConverter();
    /**
     *  @brief threeway_max Finds the maximum in a triple of floating point values. Requires fmax from cmath
     *  @param a
     *  @param b
     *  @param c
     *  @return maximum of a,b and c
     */
    static double threeway_max(double a, double b, double c);
    /**
     *  @brief threeway_min Finds the minimum in a triple of floating point values. Requires fmin from cmath
     *  @param a
     *  @param b
     *  @param c
     *  @return minimum of a,b and c
     */
    static double threeway_min(double a, double b, double c);
    /**
     *  @brief hueToRgb Used internally in hslToRgb for handling the hue value
     *  @param p
     *  @param q
     *  @param t
     *  @return p
     */
    static double hueToRgb(double p, double q, double t);
};
#endif // RGBConverter_h
