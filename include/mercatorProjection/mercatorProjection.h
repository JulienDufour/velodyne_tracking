/**
 * @file   mercatorProjection.h
 * @author DUFOUR Julien
 * @date   March, 2016
 * @brief  File containing Mercator transformation headers.
 *
 * These methods permit to obtain Mercator coordinates from latitude and longitude.
 * Also, these methods permit to obtain GPS coordinates from Mercator coordinates.
 * Takes in account the fact that the earth is not a sphere, but a spheroid.
 */

#include <math.h>

#ifndef MP_H
#define MP_H

/*
 * Mercator transformation
 * Takes in account the fact that the earth is not a sphere, but a spheroid
 */
#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0 // semi-major axis (WGS84) in meters
#define R_MINOR 6356752.314245179497563967 // Semi-minor axis (WGS84) in meters
#define ECCENT 0.081819190842622 // First eccentricity (WGS84)
#define COM (0.5 * ECCENT)

class mp{

public:
	/**
     *  @brief merc_x From longitude, compute mercator (x) for a spheroidal earth.
     *  @param lon    Longitude (in DD)  (input)
     *  @return       X in mercator PCS.
     */
	static double merc_x(double lon);

	/**
     *  @brief merc_y From longitude, compute mercator (y) for a spheroidal earth.
     *  @param lat    Latitude (in DD)  (input)
     *  @return       Y in mercator PCS.
     */
	static double merc_y (double lat);

	/**
     *  @brief merc_lon From X in mercator CS, compute longitude.
     *  @param x        X (in mercator CS).  (input)
     *  @return         Longitude (in DD).
     */
	static double merc_lon (double x);

	/**
     *  @brief merc_lat From Y in mercator CS, compute latitude.
     *  @param x        Y (in mercator CS).  (input)
     *  @return         Latitude (in DD).
     */
	static double merc_lat (double y);

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

private:
        mp();

};

#endif