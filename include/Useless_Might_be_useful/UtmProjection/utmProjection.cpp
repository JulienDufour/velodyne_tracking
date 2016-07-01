/**
 * @file   utmProjection.cpp
 * @author DUFOUR Julien
 * @date   May, 2016
 * @brief  File containing Universal Transverse Mercator transformation methods.
 *
 * These methods permit to obtain UTM coordinates with a cm accuracy from latitude and longitude.
 * Also, these methods permit to obtain GPS coordinates from UTM coordinates.
 * @see https://fr.wikipedia.org/wiki/Transverse_Universelle_de_Mercator
 */

#include <UtmProjection/utmProjection.h>

double utm::deg_rad(double ang){
    return ang * D_R;
}

double utm::rad_deg (double ang){
    return ang * R_D;
}

double utm::make_v(double lat){
    double phi = utm::deg_rad(lat);
    return 1.0 / sqrt(1 - (pow(ECCENT,2)*pow(sin(phi),2)));
}

double utm::make_A(double lat, double lon){
    double phi = utm::deg_rad(lat);
    double lambda = utm::deg_rad(lon - lambda_0);
    return lambda * cos(phi);
}

double utm::make_s(double lat){
    double phi = utm::deg_rad(lat);

    double s;
    s = ( 1.0 - (pow(ECCENT,2)/4.0) - ((3*pow(ECCENT,4))/64.0) - ((5*pow(ECCENT,6))/256.0) ) * phi;
    s -= ( ((3*pow(ECCENT,2))/8.0) + ((3*pow(ECCENT,4))/32.0) + ((45*pow(ECCENT,6))/1024.0) ) * sin(2*phi);
    s += ( ((15*pow(ECCENT,4))/256.0) + ((45*pow(ECCENT,6))/1024.0) ) * sin(4*phi);
    s -= ((35*pow(ECCENT,6))/3072.0) * sin(6*phi);

    return s;
}

double utm::make_T(double lat){
    double phi = utm::deg_rad(lat);
    return pow(tan(phi),2);
}

double utm::make_C(double lat){
    double phi = utm::deg_rad(lat);
    return ( pow(ECCENT,2) / (1-pow(ECCENT,2)) ) * pow(cos(phi),2);
}

double utm::utm_E(double lat, double lon){
    
    // Get intermediar values
    double v = utm::make_v(lat);
    double A = utm::make_A(lat, lon);
    double s = utm::make_s(lat);
    double T = utm::make_T(lat);
    double C = utm::make_C(lat);

    //Compute E
    double E = 500.0 + K_0 * R_MAJOR * v;
    E *= ( A + ( (1.0-T+C)*(pow(A,3)/6.0) ) + ( (5.0-18*T+pow(T,2))* (pow(A,5)/120.0) ) );

    return E;
}

double utm::utm_N(double lat, double lon){
    double phi = utm::deg_rad(lat);

    // Get intermediar values
    double v = utm::make_v(lat);
    double A = utm::make_A(lat, lon);
    double s = utm::make_s(lat);
    double T = utm::make_T(lat);
    double C = utm::make_C(lat);

    //Compute N
    double N = pow(A,2) / 2.0;
    N += ( 5.0-T+9.0*C+ 4.0*pow(C,2) ) * ( pow(A,4)/24.0 );
    N += ( 61.0-58.0*T+pow(T,2) ) * ( pow(A,6)/720.0 );
    N *= v*tan(phi);
    N += s;
    N = N_0 + K_0 * R_MAJOR * N;

    return N;
}

void utm::utm_E_N(double lat, double lon, double &E, double &N){
    double phi = utm::deg_rad(lat);

    // Get intermediar values
    double v = utm::make_v(lat);
    double A = utm::make_A(lat, lon);
    double s = utm::make_s(lat);
    double T = utm::make_T(lat);
    double C = utm::make_C(lat);

    //Compute E
    E = 500000.0 + K_0 * R_MAJOR * v;
    E *= ( A + ( (1.0-T+C)*(pow(A,3)/6.0) ) + ( (5.0-18*T+pow(T,2))* (pow(A,5)/120.0) ) );

    //Compute N
    N = pow(A,2) / 2.0;
    N += ( 5.0-T+9.0*C+ 4.0*pow(C,2) ) * ( pow(A,4)/24.0 );
    N += ( 61.0-58.0*T+pow(T,2) ) * ( pow(A,6)/720.0 );
    N *= v*tan(phi);
    N += s;
    N = N_0 + K_0 * R_MAJOR * N;
}