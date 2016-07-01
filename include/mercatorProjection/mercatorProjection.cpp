/**
 * @file   mercatorProjection.cpp
 * @author DUFOUR Julien
 * @date   March, 2016
 * @brief  File containing Mercator transformation methods.
 *
 * These methods permit to obtain Mercator coordinates from latitude and longitude.
 * Also, these methods permit to obtain GPS coordinates from Mercator coordinates.
 */

#include <mercatorProjection/mercatorProjection.h>

double mp::deg_rad(double ang){
    return ang * D_R;
}

double mp::merc_x (double lon){
    return R_MAJOR * mp::deg_rad(lon);
}

double mp::merc_y (double lat){
    lat = fmin (89.5, fmax (lat, -89.5));
    double phi = mp::deg_rad(lat);
    double sinphi = sin(phi);
    double con = ECCENT * sinphi;
    con = pow((1.0 - con) / (1.0 + con), COM);
    double ts = tan(0.5 * (M_PI * 0.5 - phi)) * con;
    return 0 - R_MAJOR * log(ts);
}

double mp::rad_deg (double ang) {
    return ang * R_D;
}

double mp::merc_lon (double x) {
    return mp::rad_deg(x) / R_MAJOR;
}

double mp::merc_lat (double y) {
    double ts = exp ( -y / R_MAJOR);
    double phi = M_PI_2 - 2 * atan(ts);
    double dphi = 1.0;
    int i;
    for (i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
            double con = ECCENT * sin (phi);
            dphi = M_PI_2 - 2 * atan (ts * pow((1.0 - con) / (1.0 + con), COM)) - phi;
            phi += dphi;
    }
    return mp::rad_deg (phi);
}