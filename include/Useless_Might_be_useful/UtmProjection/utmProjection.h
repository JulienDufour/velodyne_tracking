/**
 * @file   utmProjection.h
 * @author DUFOUR Julien
 * @date   May, 2016
 * @brief  File containing Universal Transverse Mercator transformation headers.
 *
 * These methods permit to obtain UTM coordinates with a cm accuracy from latitude and longitude.
 * Also, these methods permit to obtain GPS coordinates from UTM coordinates.
 * @see https://fr.wikipedia.org/wiki/Transverse_Universelle_de_Mercator
 */

#include <math.h>

#ifndef UTM_H
#define UTM_H

/*
 * Universal Transverse Mercator transformation
 * Give CS in meters with precision of cm
 */
#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0 // semi-major axis (WGS84) in meters
#define R_MINOR 6356752.314245179497563967 // Semi-minor axis (WGS84) in meters
#define ECCENT 0.081819190842622 // First eccentricity (WGS84)
#define K_0 0.9996
#define N_0 0 //It is for northern hemisphere. For southern hemisphere it is 10000*10³ meters.
#define lambda_0 9.0 //central meridian UTM zone 32

class utm{

public:

	/**
     *  @brief deg_rad Does degree to radian conversion.
     *  @param ang     Angle (in Degree).  (input)
     *  @return        Angle (in Radian).
     */
	static double deg_rad(double ang);

	/**
     *  @brief deg_rad Does radian to degree conversion.
     *  @param ang     Angle (in Radian).  (input)
     *  @return        Angle (in Degree).
     */
	static double rad_deg (double ang);

     /**
     *  @brief utm_E  From latitude and longitude, compute E in meters.
     *  @param lat    Latitude (in DD)  (input)
     *  @param lon    Longitude (in DD) (input)
     *  @return       E in meters.
     */
     static double utm_E(double lat, double lon);

     /**
     *  @brief utm_N  From latitude and longitude, compute N in meters.
     *  @param lat    Latitude (in DD)  (input)
     *  @param lon    Longitude (in DD) (input)
     *  @return       N in meters.
     */
     static double utm_N(double lat, double lon);

     /**
     *  @brief utm_E_N  From latitude and longitude, compute N and E in meters.
     *  @param lat      Latitude (in DD)  (input)
     *  @param lon      Longitude (in DD) (input)
     */
     static void utm_E_N(double lat, double lon, double &E, double &N);

private:
     utm();

     // For the next functions see : https://fr.wikipedia.org/wiki/Transverse_Universelle_de_Mercator
     
     /**
     *  @brief make_v From latitude, compute v(phi).
     *  @param lat    Latitude (in DD)  (input)
     *  @return       v(phi).
     */
     static double make_v(double lat);

     /**
     *  @brief make_A From latitude and longitude, compute A.
     *  @param lat    Latitude (in DD)  (input)
     *  @param lon    Longitude (in DD) (input)
     *  @return       A.
     */
     static double make_A(double lat, double lon);

     /**
     *  @brief make_s From latitude, compute s(phi).
     *  @param lat    Latitude (in DD)  (input)
     *  @return       s(phi).
     */
     static double make_s(double lat);

     /**
     *  @brief make_T From latitude, compute T.
     *  @param lat    Latitude (in DD)  (input)
     *  @return       T.
     */
     static double make_T(double lat);

     /**
     *  @brief make_C From latitude, compute C.
     *  @param lat    Latitude (in DD)  (input)
     *  @return       C.
     */
     static double make_C(double lat);

};

#endif