/***************************************************************************
 *  Copyright (C) 2016  by DUFOUR Julien                                   *
 *                                                                         *
 *  This program is free software: you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           * 
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

/**
 * @file   velocityProfile.h
 * @author DUFOUR Julien
 * @date   June, 2016
 * @brief  File containing velocity profile class headers.
 *
 * This class permits to obtain a collection of three velocity profiles (defensive, normal and sporty) from the path coordinates.
 *
 * To create the velocity profiles the needs are :
 * _ Coordinates of one path.
 * _ The speed limits at each coordinate.
 * _ The traffic rule at each coordinate.
 *
 * This class proposes to apply many constraints which are :
 * _ Traffic rules
 * _ Lateral acceleration and curvature
 * _ Acceleration
 * _ Deceleration
 * _ Intelligent Driver Model (IDM) 
 */

#include <algorithm>    // std::find

#include <glm/glm.hpp> // Useful for cubic spline interpolation
#include <spline/splineInterpolation.h> // Cubic spline interpolation
#include <Lowess/lowess.h> // Curve regression

#ifndef VELOCITYPROFILE_H
#define VELOCITYPROFILE_H

/**
 *  @brief Class which regroup all usefull function to create one or more velocity profile(s) from one path (in meters).
 */
class velocityProfile{

	/*-----------------------------------------------------------------------------
       *  Attributes
       *-----------------------------------------------------------------------------
       */

public:

private:
	std::vector<double>      _xPath;             /*!< List of X coordinates which represent the path (in Mercator CS in meters).*/
	std::vector<double>      _yPath;             /*!< List of Y coordinates which represent the path (in Mercator CS in meters).*/
	std::vector<double>      _speedLimits;       /*!< Speed limitation at each (X,Y) coordinates for the path (in m/s).*/
	std::vector<std::string> _trafficRules;      /*!< Traffic rule at each (X,Y) coordinates for the path.*/
	std::vector<double>      _curvatureProfile;  /*!< Curvature at each (X,Y) coordinates for the path (1/m).*/
	std::vector<double>      _distanceProfile;   /*!< Temporal axis of the path. Represents at each (X,Y), the distance (in meters) (X0,Y0) -> (X,Y).*/
	std::vector<std::vector<std::vector<double>>> _velocityProfiles; /*!< Velocity profiles (defensive, normal, sporty) from the curvature profile of the path. [Case][Profile][idx_distance]*/

	/*-----------------------------------------------------------------------------
       *  Methods
       *-----------------------------------------------------------------------------
       */
     
public:
	velocityProfile(){};

	/**
       *  @brief init          Initialize velocityProfile object.
       *  @param xPath         List of X coordinates for the input path (in Mercator CS in meters).     (input)
       *  @param yPath         List of Y coordinates for the input path (in Mercator CS in meters).     (input)
       *  @param speedLimits   Speed limitation at each (X,Y) coordinates for the input path (in Km/h). (input)
       *  @param trafficRules  Traffic rule at each (X,Y) coordinates for the input path.               (input)
       *  @return              False if any problem occurs.
       */
	bool init(std::vector<double> xPath, std::vector<double> yPath, std::vector<double> speedLimits, std::vector<std::string> trafficRules);

	/**
       *  @brief init          Initialize velocityProfile object.
       *  @param path          List of pair <X,Y> which represent the coordinates for the input path (in Mercator CS in meters).  (input)
       *  @param speedLimits   Speed limitation at each (X,Y) coordinates for the input path (in Km/h).                           (input)
       *  @param trafficRules  Traffic rule at each (X,Y) coordinates for the input path.                                         (input)
       *  @return              False if any problem occurs.
       */
	bool init(std::vector<std::pair<double,double>> path, std::vector<double> speedLimits, std::vector<std::string> trafficRules);

	/**
       *  @brief get_xPath  Get _xPath attribute.
       *  @return           List of X coordinates which represent the path (in Mercator CS in meters).
       */
	std::vector<double> get_xPath(){return _xPath;}

	/**
       *  @brief get_yPath  Get _yPath attribute.
       *  @return           List of Y coordinates which represent the path (in Mercator CS in meters).
       */
	std::vector<double> get_yPath(){return _yPath;}

	/**
       *  @brief get_speedLimits  Get _speedLimits attribute.
       *  @return                 Speed limitation at each (X,Y) coordinates for the path (in m/s).
       */
	std::vector<double> get_speedLimits(){return _speedLimits;}

	/**
       *  @brief get_trafficRules  Get _trafficRules attribute.
       *  @return                  Traffic rule at each (X,Y) coordinates for the path.
       */
	std::vector<std::string> get_trafficRules(){return _trafficRules;}

	/**
       *  @brief get_curvatureProfile  Get _curvatureProfile attribute.
       *  @return                      Curvature at each (X,Y) coordinates for the path (1/m).
       */
	std::vector<double> get_curvatureProfile(){return _curvatureProfile;}

	/**
       *  @brief get_distanceProfile  Get _distanceProfile attribute.
       *  @return                     Temporal axis of the path. Represents at each (X,Y), the distance (in meters) (X0,Y0) -> (X,Y).
       */
	std::vector<double> get_distanceProfile(){return _distanceProfile;}

	/**
       *  @brief get_velocityProfiles  Get _velocityProfiles attribute.
       *  @return                      Velocity profiles (defensive, normal, sporty) from the curvature profile of the path for each case provided by traffic rules.
       */
	std::vector<std::vector<std::vector<double>>> get_velocityProfiles(){return _velocityProfiles;}
	
	/**
       *  @brief reSamplePath             Re sample path represented by (_xPath,_yPath) following "minDistBetweenTwoPoints" parameter.
       *  @param minDistBetweenTwoPoints  Minimal distance between two points in the final path (in meters). Default value is 15.0 meters.  (input)
 	 *  @return                         False if any problem occurs.
       */
	bool reSamplePath(double minDistBetweenTwoPoints = 15.0);

	/**
       *  @brief smoothHardSoftTurn  Smooth turns into path represented by (_xPath,_yPath).
       *					Hard turn represents a turn around 90 degree.
       *                             Soft turn represents the other turn.
       *  @param minHardTurnAngle    Minimal bound which define one hard turn (in degree). Default value is 65.0 degrees.   (input)
       *  @param maxHardTurnAngle    Maximal bound which define one hard turn (in degree). Default value is 115.0 degrees.  (input)
       *  @param minSoftTurnAngle    Minimal bound which define one soft turn (in degree). Default value is 20.0 degrees.   (input)
       *  @param maxSoftTurnAngle    Maximal bound which define one soft turn (in degree). Default value is 160.0 degrees.  (input)
 	 *  @return                    False if any problem occurs.
       */
	bool smoothHardSoftTurn(double minHardTurnAngle = 65.0, double maxHardTurnAngle = 115.0, double minSoftTurnAngle = 20.0, double maxSoftTurnAngle = 160.0);

	/**
       *  @brief setCurvatureProfile  Create the curved path and the curvature profile from the path represented by (_xPath,_yPath).
       *  @param n                    Number of desired samples in curvature profile. For good result, assure that it is bigger than the size of the path.  (input)  
 	 *  @return                     False if any problem occurs.
       */
	bool setCurvatureProfile(int n);

	/**
       *  @brief smoothCurvatureProfile  Smooth curvature profile from the curvature represented by _curvatureProfile.
       *  @param amountOfSmoothing       Specifies the amount of smoothing for the curvature profile. 
       *                                 For more information, see documentation for parameter "F" in "Lowess/lowess.cpp" file. Default value is 0.1.  (input)
 	 *  @return                        False if any problem occurs.    
       */
	bool smoothCurvatureProfile(double amountOfSmoothing = 0.1);

	/**
       *  @brief setVelocityProfiles   Create 3 velocity profiles (defensive, normal, sporty) from the speed limits of the path.
       *  @param maxSpeedPercentP1     Percentage of law speed for the defensive profile, p1, (in %). Default value is 80.0 %.  (input)
       *  @param maxSpeedPercentP2     Percentage of law speed for the normal profile, p2, (in %). Default value is 90.0 %.     (input)
       *  @param maxSpeedPercentP3     Percentage of law speed for the sporty profile, p3, (in %). Default value is 100.0 %.    (input)
 	 *  @return                      False if any problem occurs.
       */
	bool setVelocityProfiles(double maxSpeedPercentP1 = 80.0, double maxSpeedPercentP2 = 90.0, double maxSpeedPercentP3 = 100.0);

	/**
       *  @brief addLatAccelerationConstraints   Add curvature and lateral accelaration constraints into the velocity profiles (defensive, normal, sporty). 
       *  @param aLatP1                          Maximal lateral acceleration for the defensive profile, p1, (in m/s²). Default value is 2.0 m/s².  (input)
       *  @param aLatP2                          Maximal lateral acceleration for the normal profile, p2, (in m/s²). Default value is 2.75 m/s².    (input)
       *  @param aLatP3                          Maximal lateral acceleration for the sporty profile, p3, (in m/s²). Default value is 3.5 m/s².     (input)
 	 *  @return                                False if any problem occurs.
       */
	bool addLatAccelerationConstraints(double aLatP1 = 2.0, double aLatP2 = 2.75, double aLatP3 = 3.5);

	/**
       *  @brief addAccelerationConstraints   Add the accelaration constraints into the velocity profiles (defensive, normal, sporty).
       *  @param aP1                          Maximum acceleration parameter for the defensive profile, p1, (in m/s²). Default value is 1.5 m/s².  (input)
       *  @param aP2                          Maximum acceleration parameter for the normal profile, p2, (in m/s²). Default value is 2.0 m/s².     (input)
       *  @param aP3                          Maximum acceleration parameter for the sporty profile, p3, (in m/s²). Default value is 2.5 m/s².     (input)
 	 *  @return                             False if any problem occurs.
       */
	bool addAccelerationConstraints(double aP1 = 1.5, double aP2 = 2.0, double aP3 = 2.5);

	/**
       *  @brief addDecelerationConstraints   Add the deceleration constraints into the velocity profiles (defensive, normal, sporty).
       *  @param gP1                          Minimal velocity gradient for the defensive profile, p1, (in 1/s). Default value is 0.15 1/s.  (input)
       *  @param gP2                          Minimal velocity gradient for the normal profile, p2, (in 1/s). Default value is 0.20 1/s.     (input)
       *  @param gP3                          Minimal velocity gradient for the sporty profile, p3, (in 1/s). Default value is 0.25 1/s.     (input)
 	 *  @return                             False if any problem occurs.
       */
	bool addDecelerationConstraints(double gP1 = 0.15, double gP2 = 0.20, double gP3 = 0.25);

	/**
       *  @brief addTrafficRulesConstraints   Add the traffic rules constraints into the velocity profiles (defensive, normal, sporty).
       *                                      This method can create until 3^(Nb_traffic_rule) cases for the 3 veloctity profiles (defensive, normal, sporty).
       *  @param stopDistance                 Stopping distance before one traffic rule (in meters).                                                              (input)
       *  @param maxSpeedPercentP1            Percentage of law speed at intersections (30 Km/h) for the defensive profile, p1, (in %). Default value is 80.0 %.  (input)
       *  @param maxSpeedPercentP2            Percentage of law speed at intersections (30 Km/h) for the normal profile, p2, (in %). Default value is 90.0 %.     (input)
       *  @param maxSpeedPercentP3            Percentage of law speed at intersections (30 Km/h) for the sporty profile, p3, (in %). Default value is 100.0 %.    (input)
       *  @param gP1                          Minimal velocity gradient for the defensive profile, p1, (in 1/s). Default value is 0.15 1/s.                       (input)
       *  @param gP2                          Minimal velocity gradient for the normal profile, p2, (in 1/s). Default value is 0.20 1/s.                          (input)
       *  @param gP3                          Minimal velocity gradient for the sporty profile, p3, (in 1/s). Default value is 0.25 1/s.                          (input)
 	 *  @return                             False if any problem occurs.
       */
     bool addTrafficRulesConstraints(double stopDistance, double maxSpeedPercentP1 = 80.0, double maxSpeedPercentP2 = 90.0, double maxSpeedPercentP3 = 100.0, double gP1 = 0.15, double gP2 = 0.20, double gP3 = 0.25);

	/**
       *  @brief cleanVelocityProfiles  Delete same case in _velocityProfiles following an acceptable difference parameter.
       *  @param equalDiffPrecent       Number of acceptable difference (in %) during the equality test. Default value is 0 %  (input)  
 	 *  @return                       False if any problem occurs.
       */
	bool cleanVelocityProfiles(double equalDiffPrecent = 0.0);

      /**
       *  @brief addIdmPrecedingObjectConstraints   Add Driver Behavior Model (IDM with following situation) constraints into the velocity profiles (defensive, normal, sporty). 
       *  @param actualGap                          Effective actual gap between current object and preceding object.                                    (input)
       *  @param objVelocity                        Velocity of current object (in m/s).                                                                 (input)
       *  @param preObjVelocity                     Velocity of preceding object (in m/s).                                                               (input)
       *  @param currentDist                        Reference distance (in _distanceProfile) of current object for this velocityProfile (in m).          (input)
       *  @param aP1                                Maximum acceleration parameter for the defensive profile, p1, (in m/s²). Default value is 1.5 m/s².  (input)
       *  @param aP2                                Maximum acceleration parameter for the normal profile, p2, (in m/s²). Default value is 2.0 m/s².     (input)
       *  @param aP3                                Maximum acceleration parameter for the sporty profile, p3, (in m/s²). Default value is 2.5 m/s².     (input)
       *  @param aExpo                              Acceleration exponent. Default value is 4.                                                           (input)
       *  @param comfDeceleration                   Comf. deceleration (in m/s²). Default value is 3.0 m/s².                                             (input)
       *  @param d0                                 Min. gap to leading vehicule (in m). Default value is 2.0 m.                                         (input)
       *  @param timeGap                            Time gap to leading vehicule (in s). Default value is 0.8 s.                                         (input)
       *  @return                                   False if any problem occurs.
       */
     bool addIdmPrecedingObjectConstraints(double actualGap, double objVelocity, double preObjVelocity, double currentDist,
     double aP1 = 1.5, double aP2 = 2.0, double aP3 = 2.5, double aExpo = 4.0, double comfDeceleration = 3.0, double d0 = 2.0, double timeGap = 0.8);

      /**
       *  @brief addIdmAccelerationConstraints   Add Driver Behavior Model (IDM without following situation) constraints into the velocity profiles (defensive, normal, sporty). 
       *  @param objVelocity                     Velocity of current object (in m/s).                                                                 (input)
       *  @param currentDist                     Reference distance (in _distanceProfile) of current object for this velocityProfile (in m).          (input)
       *  @param aP1                             Maximum acceleration parameter for the defensive profile, p1, (in m/s²). Default value is 1.5 m/s².  (input)
       *  @param aP2                             Maximum acceleration parameter for the normal profile, p2, (in m/s²). Default value is 2.0 m/s².     (input)
       *  @param aP3                             Maximum acceleration parameter for the sporty profile, p3, (in m/s²). Default value is 2.5 m/s².     (input)
       *  @param aExpo                           Acceleration exponent. Default value is 4.                                                           (input)
       *  @return                                False if any problem occurs.
       */
     bool addIdmAccelerationConstraints(double objVelocity, double currentDist, double aP1 = 1.5, double aP2 = 2.0, double aP3 = 2.5, double aExpo = 4.0);

      /**
       *  @brief getVelocity                  Get velocity for one case and one profile among defensive, normal and sporty at one particular distance on the path.
       *  @param desiredCase                  Desired case among all profiles "package" (defensive, normal, sporty) contained in _velocityProfiles.  (input)
       *  @param desiredProfile               Desired profile : defensive (0), normal (1) or sporty (2).                                             (input)
       *  @param atDistance                   Desired distance. Must be included in _distanceProfile.                                                (input)
       *  @param velocity                     Velocity of desired profile in the desired case at desired distance.                                   (output)
       *  @return                             False if any problem occurs.
       */
     bool getVelocity(int desiredCase, int desiredProfile, double atDistance, double &velocity);

private:

};

#endif