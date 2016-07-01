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
 * @brief  File containing velocity profile class methods.
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

#include <velocityProfile/velocityProfile.h>

bool velocityProfile::init(std::vector<double> xPath, std::vector<double> yPath, std::vector<double> speedLimits, std::vector<std::string> trafficRules){
	
	if(xPath.size() == yPath.size()
	   && xPath.size() == speedLimits.size()
	   && xPath.size() == trafficRules.size()){
		
		_xPath = xPath;
		_yPath = yPath;
		_trafficRules = trafficRules;

		// Speed limits : Km/h -> m/s
		for (int i = 0; i < static_cast<int>(speedLimits.size()); ++i)
		{
			_speedLimits.push_back(speedLimits[i]/3.6);
		}

		return true;
	}
	else{
		std::cerr<<"Error during initialization of velocityProfile : Input vector do not have the same size"<<std::endl;
        return false;
	}
}

bool velocityProfile::init(std::vector<std::pair<double,double>> path, std::vector<double> speedLimits, std::vector<std::string> trafficRules){
	
	if(path.size() == speedLimits.size() 
		&& path.size() == trafficRules.size()){

		for (int i = 0; i < static_cast<int>(path.size()); ++i)
		{
			_xPath.push_back(path[i].first);
			_yPath.push_back(path[i].second);
			_speedLimits.push_back(speedLimits[i]/3.6); // Speed limits : Km/h -> m/s

		}

		_trafficRules = trafficRules;

		return true;
	}
	else{
		std::cerr<<"Error during initialization of velocityProfile : Input vector do not have the same size"<<std::endl;
        return false;
	}
}

bool velocityProfile::reSamplePath(double maxDistBetweenTwoPoints){

	if(_xPath.empty() || _yPath.empty() || _speedLimits.empty() || _trafficRules.empty()){
		std::cerr<<"reSamplePath cannot be used. velocityProfile must be initialized. Please use init methods before."<<std::endl;
        return false;
	}
    
    std::vector<double> xPath_tmp = std::vector<double>{_xPath[0]};
    std::vector<double> yPath_tmp = std::vector<double>{_yPath[0]};
    std::vector<double> speedLimits_tmp = std::vector<double>{_speedLimits[0]};
    std::vector<std::string> trafficRules_tmp = std::vector<std::string>{_trafficRules[0]};

    for(int i=1 ; i < static_cast<int>(_xPath.size()); i++){

        // If this node is not in the output list, add it and re-sample data
        if(std::find(xPath_tmp.begin(), xPath_tmp.end(), _xPath[i]) == xPath_tmp.end() && std::find(yPath_tmp.begin(), yPath_tmp.end(), _yPath[i]) == yPath_tmp.end()){
        
            // Distance between next and precedent node
            double distanceBetweenNode = sqrt(pow(_xPath[i]-xPath_tmp.back(),2)+pow( _yPath[i]-yPath_tmp.back(),2));

            // Coeff which represents the number of necessary cut
            int nbCut = ceil(distanceBetweenNode/maxDistBetweenTwoPoints);

            double addOnX = (_xPath[i]-xPath_tmp.back()) * ( (1.0/nbCut) );
            double addOnY = (_yPath[i]-yPath_tmp.back()) * ( (1.0/nbCut) );
            
            for (int idxCut = 1; idxCut <= nbCut; ++idxCut)
            {
                xPath_tmp.push_back(xPath_tmp.back() + addOnX);
                yPath_tmp.push_back(yPath_tmp.back() + addOnY);

                if(_speedLimits[i] == -1) speedLimits_tmp.push_back(speedLimits_tmp.back());
                else speedLimits_tmp.push_back(_speedLimits[i]);
            
                if(idxCut == nbCut) trafficRules_tmp.push_back(_trafficRules[i]);
                else trafficRules_tmp.push_back("no");
            }
        }
    }

    // Update velocityProfile attributes
    _xPath = xPath_tmp;
    _yPath = yPath_tmp;
    _speedLimits = speedLimits_tmp;
    _trafficRules = trafficRules_tmp;

    return true;
}

bool velocityProfile::smoothHardSoftTurn(double minHardTurnAngle, double maxHardTurnAngle, double minSoftTurnAngle, double maxSoftTurnAngle){

	if(_xPath.empty() || _yPath.empty()){
		std::cerr<<"smoothHardSoftTurn cannot be used. velocityProfile must be initialized. Please use init methods before."<<std::endl;
        return false;
	}

	std::vector<double> xPath_tmp = _xPath;
    std::vector<double> yPath_tmp = _yPath;

    for (int i = 1; i < static_cast<int>(xPath_tmp.size())-1; i++)
    {
    	//--------------------
	    // First step : Define the angle formed by the way.
	    // It is represented by the angle between the two vectors path(i-1)->path(i), path(i)->path(i+1).
	    //--------------------

        // Precedent vector <=> path(i-1)->path(i) (V1)
        double dxV1 = xPath_tmp[i] - xPath_tmp[i-1];
        double dyV1 = yPath_tmp[i] - yPath_tmp[i-1];
        double distanceV1 = sqrt(pow(dyV1,2)+pow(dxV1,2));

        dxV1 = dxV1/distanceV1;
        dyV1 = dyV1/distanceV1;

        // Next vector <=> path(i)->path(i+1) (V2) 
        double dxV2 = xPath_tmp[i+1] - xPath_tmp[i];
        double dyV2 = yPath_tmp[i+1] - yPath_tmp[i];
        double distanceV2 = sqrt(pow(dyV2,2)+pow(dxV2,2));

        dxV2 = dxV2/distanceV2;
        dyV2 = dyV2/distanceV2;

        // Angle between (V1) and (V2), normalize in [-180.0 180.0] degree 
        double angleVect = atan2(dyV2,dxV2)-atan2(dyV1,dxV1);
        if(angleVect < 0) angleVect += 2 * M_PI;
        angleVect = (angleVect*(180.0 / M_PI)) - 180.0;

        //--------------------
	    // Second step : Smooth "hard" and "soft" turn and update velocityProfile attributes "_xPath" and "_yPath".
	    //--------------------

        // "Hard" turn : Move path(i) point to the center of triangle formed by path(i-1), path(i) and path(i+1).
        if(fabs(angleVect) >= minHardTurnAngle && fabs(angleVect) <= maxHardTurnAngle){
            _xPath[i] = (xPath_tmp[i-1] + xPath_tmp[i] + xPath_tmp[i+1])/3.0;
            _yPath[i] = (yPath_tmp[i-1] + yPath_tmp[i] + yPath_tmp[i+1])/3.0;
        }
        // "soft" turn : Move path(i) point to the center of the median which have path(i) as top and (path(i-1),path(i+1)) as side.
        else if(fabs(angleVect) >= minSoftTurnAngle && fabs(angleVect) <= maxSoftTurnAngle){

            double dxV3 = ((xPath_tmp[i+1]+xPath_tmp[i-1])/2.0) - xPath_tmp[i];
            double dyV3 = ((yPath_tmp[i+1]+yPath_tmp[i-1])/2.0) - yPath_tmp[i];

            _xPath[i] = xPath_tmp[i] + dxV3/2.0;
            _yPath[i] = yPath_tmp[i] + dyV3/2.0;

        }
    }

    return true;

}

bool velocityProfile::setCurvatureProfile(int n){

	if(_xPath.empty() || _yPath.empty() || _speedLimits.empty() || _trafficRules.empty()){
		std::cerr<<"setCurvatureProfile cannot be used. velocityProfile must be initialized. Please use init methods before."<<std::endl;
        return false;
	}

	//--------------------------------------------
    // First step : Create spline from the path defined by (_xPath,_yPath).
	//--------------------------------------------

    std::vector<float> times;
    std::vector<glm::vec2> points;

    // Make a set of points according to cubic spline interpolation methods in splineInterpolation.h.
    times.push_back(0.0);
    points.push_back(glm::vec2(_xPath[0], _yPath[0]));

    for (int i = 1; i < static_cast<int>(_xPath.size()); i++)
    {
        //times.push_back( i );
        times.push_back( times.back() + sqrt(pow(_xPath[i]-_xPath[i-1],2)+pow(_yPath[i]-_yPath[i-1],2)) );
        points.push_back(glm::vec2(_xPath[i], _yPath[i]));
    }

    // Create the spline interpolating the position over time.
    Spline<float, glm::vec2> sp(times, points);

	//--------------------------------------------
    // Second step : Create curvature profile from the spline.
    //               Also, create _distanceProfile and re sample _xPath, _yPath, _speedLimits, _trafficRules.
	//--------------------------------------------

    // Assure that ("output") attributes are empty.
    _xPath.clear();
    _yPath.clear();
    _distanceProfile.clear();
    _curvatureProfile.clear();

    // Initialization : t=0

    /* Speed limits update */
    std::vector<double> speedLimits_tmp = std::vector<double>{_speedLimits[0]};

    /* Traffic rules update */
    std::vector<std::string> trafficRules_tmp = std::vector<std::string>{_trafficRules[0]};

    /* Distance update */
    _distanceProfile.push_back(0.0);
    
    /* Path update */
    glm::vec2 splineValue(sp.interpolate(0.0));
    _xPath.push_back(splineValue[0]);
    _yPath.push_back(splineValue[1]);
    
    /* Curvature update */
    glm::vec2 firstDeriv(sp.firstDerivate(0.0));
    glm::vec2 secondDeriv(sp.secondDerivate(0.0));
    
    double curv = fabs( firstDeriv[0]*secondDeriv[1] -  firstDeriv[1]*secondDeriv[0]);
    curv /= pow( (pow(firstDeriv[0],2)+pow(firstDeriv[1],2)) ,3.0/2.0);
    
    _curvatureProfile.push_back(curv); 

    // Process
    int trafficRulesIdx = 1;
    for (float currentTime = times.back()/static_cast<double>(n); currentTime <= times.back(); currentTime += times.back()/static_cast<double>(n))
    {
	    glm::vec2 splineValue(sp.interpolate(currentTime));

	    /* Distance update */
    	_distanceProfile.push_back( _distanceProfile.back() + sqrt(pow(splineValue[0]-_xPath.back(),2)+pow(splineValue[1]-_yPath.back(),2)) );

    	/* Path update */
    	_xPath.push_back(splineValue[0]);
	    _yPath.push_back(splineValue[1]);

	    /* Curvature update */
        glm::vec2 firstDeriv(sp.firstDerivate(currentTime));
        glm::vec2 secondDeriv(sp.secondDerivate(currentTime));

        curv = fabs( firstDeriv[0]*secondDeriv[1] -  firstDeriv[1]*secondDeriv[0]);
        curv /= pow( (pow(firstDeriv[0],2)+pow(firstDeriv[1],2)) ,3.0/2.0); 

        _curvatureProfile.push_back(curv);

        /* Speed limits update */
		speedLimits_tmp.push_back(_speedLimits[trafficRulesIdx]);

        /* Traffic rules update */
        if(currentTime >= times[trafficRulesIdx]){
        	trafficRules_tmp.push_back(_trafficRules[trafficRulesIdx]);
        	trafficRulesIdx++;
        }
        else trafficRules_tmp.push_back("no");
    }

    // Update _trafficRules and _speedLimits attributes
    _speedLimits = speedLimits_tmp;
    _trafficRules = trafficRules_tmp;

    return true;
}

bool velocityProfile::smoothCurvatureProfile(double amountOfSmoothing){

	if(_curvatureProfile.empty()){
		std::cerr<<"smoothCurvatureProfile cannot be used. _curvatureProfile must be initialized. Please use setCurvatureProfile method before."<<std::endl;
        return false;
	}

	//Curvature lowess
    std::vector<double> curvatureProfileOrigin = _curvatureProfile;
    _curvatureProfile.clear();
    lowess(_distanceProfile, curvatureProfileOrigin, amountOfSmoothing, 2, _curvatureProfile);

    // Correct Curvature profile (values < 0.0)
    for (int i = 0; i < static_cast<int>(_curvatureProfile.size()); ++i)
    {
        // smooth/correct curve profile (values < 0.0)
        if(_curvatureProfile[i]<0.0) _curvatureProfile[i]=0.0;
    }

    return true;
}

bool velocityProfile::setVelocityProfiles(double maxSpeedPercentP1, double maxSpeedPercentP2, double maxSpeedPercentP3){

	// False return test is on _curvatureProfile because setCurvatureProfile re-samples _speedLimits.
	if(_curvatureProfile.empty()){
		std::cerr<<"setVelocityProfiles cannot be used. _curvatureProfile must be initialized. Please use setCurvatureProfile method before."<<std::endl;
        return false;
	}

	// Assure that "output", _velocityProfile, is empty.
	_velocityProfiles.clear();
	_velocityProfiles.push_back(std::vector<std::vector<double>>{});
	_velocityProfiles[0].push_back(std::vector<double>{}); // P1
	_velocityProfiles[0].push_back(std::vector<double>{}); // P2
	_velocityProfiles[0].push_back(std::vector<double>{}); // P3

	for (int i = 0; i < static_cast<int>(_speedLimits.size()); ++i)
	{
		// Profil 1 : Defensive -- Max speed : maxSpeedPercentP1 (default : 80%) of _speedLimits(s)
		_velocityProfiles[0][0].push_back( _speedLimits[i] * (maxSpeedPercentP1/100.0) );

		// Profil 2 : Normal -- Max speed : maxSpeedPercentP2 (default : 90%) of _speedLimits(s)
		_velocityProfiles[0][1].push_back( _speedLimits[i] * (maxSpeedPercentP2/100.0) );

		// Profil 3 : Sporty -- Max speed : maxSpeedPercentP3 (default : 100%) of _speedLimits(s)
		_velocityProfiles[0][2].push_back( _speedLimits[i] * (maxSpeedPercentP3/100.0) );
	}

	return true;
}

bool velocityProfile::addTrafficRulesConstraints(double stopDistance, double maxSpeedPercentP1, double maxSpeedPercentP2, double maxSpeedPercentP3, double gP1, double gP2, double gP3){

	if(_velocityProfiles.empty()){
		std::cerr<<"addTrafficRulesConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfile method before."<<std::endl;
        return false;
	}

	// Go through the path and look for special traffic rules
	for (int i = 0; i < static_cast<int>(_distanceProfile.size()); ++i)
	{
		if(_trafficRules[i].compare("give_way") == 0
			|| _trafficRules[i].compare("traffic_signals") == 0
			||_trafficRules[i].compare("crossing") == 0){

			// For each existing case for the velocity profiles create two new possibilities : 
			// . v = 0.0.
			// . v slow (percentage of the reference law speed : 30 km/h) according to deceleration.

			/* Find index of stop position in _distanceProfile according to stopDistance */
			int idxStop = i;
			while(_distanceProfile[idxStop] > _distanceProfile[i]-stopDistance && idxStop > 0) idxStop--;
			
			/* Fix the stop criteria (existing case in velocity profiles) */
			int sizeVeloProfilesBeforeAdd = static_cast<int>(_velocityProfiles.size());
			/* For each case */
			for (int j = 0; j < sizeVeloProfilesBeforeAdd; ++j)
			{
				/* Create two copy of current profiles (defensive, normal, sporty) */
				std::vector<std::vector<double>> vStopProfile = _velocityProfiles[j];
				std::vector<std::vector<double>> vSlow = _velocityProfiles[j];

				/* Update vStopProfile for each profile and add this case, at the end of _velocityProfiles */
				for (int profile = 0; profile < static_cast<int>(vStopProfile.size()); ++profile)
					vStopProfile[profile][idxStop] = 0.0;
				_velocityProfiles.push_back(vStopProfile);

				/* From idxStop to begining look for vSlow index (for each profile (defensive, normal, sporty)) */
				std::vector<double> gP = std::vector<double>{gP1, gP2, gP3};
				std::vector<double> maxSpeed = std::vector<double>{(30.0/3.6)*(maxSpeedPercentP1/100.0), (30.0/3.6)*(maxSpeedPercentP2/100.0), (30.0/3.6)*(maxSpeedPercentP3/100.0)};
				int idxSlowSporty;

				/* Update vStopProfile for each profile and add this case, at the end of _velocityProfiles */
				for (int profile = 2; profile >= 0; --profile)
				{
					int idxSlow = idxStop;
					while(vStopProfile[profile][idxSlow] < maxSpeed[profile] && idxSlow > 0){
						vStopProfile[profile][idxSlow-1] = vStopProfile[profile][idxSlow] + gP[profile]*(_distanceProfile[idxSlow]-_distanceProfile[idxSlow-1]);
						idxSlow--;
					}

					/* To better understand what it happen here (role of idxSlowSporty value), look lower horizontal line on velocityProfile/idxSlowSporty.png */
					if(profile == 2) idxSlowSporty = idxSlow;

					for (int idxSlowProfile = idxSlow; idxSlowProfile <= idxSlowSporty; ++idxSlowProfile)
					{
						vSlow[profile][idxSlowProfile] = maxSpeed[profile];
					}

				}
				_velocityProfiles.push_back(vSlow);
			}
		}
		else if(_trafficRules[i].compare("stop") == 0){

			// For each existing case for the velocity profiles fix v = 0.0 for the stop position.

			/* Find index of stop position in _distanceProfile according to stopDistance */
			int idxStop = i;
			while(_distanceProfile[idxStop] > _distanceProfile[i]-stopDistance && idxStop >= 0) idxStop--;

			/* For all existing cases and each velocity profile set v = 0.0 at currentDist - stopDistance */
			for (int j = 0; j < static_cast<int>(_velocityProfiles.size()); ++j)
			{
				for (int profile = 0; profile < static_cast<int>(_velocityProfiles[j].size()); ++profile)
					_velocityProfiles[j][profile][idxStop] = 0.0;
			}
		}
	}

	// Delete same case in _velocityProfiles
	velocityProfile::cleanVelocityProfiles();

	return true;
}

bool velocityProfile::addLatAccelerationConstraints(double aLatP1, double aLatP2, double aLatP3){

	if(_velocityProfiles.empty()){
		std::cerr<<"addLatAccelerationConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfile method before."<<std::endl;
        return false;
	}

	for (int j = 0; j < static_cast<int>(_velocityProfiles.size()); ++j)
	{
		for (int i = 0; i < static_cast<int>(_velocityProfiles[j][0].size()); ++i)
		{
			// Profil 1 : Defensive
			_velocityProfiles[j][0][i] = std::min( sqrt(aLatP1/_curvatureProfile[i]),  _velocityProfiles[j][0][i]);

			// Profil 2 : Normal
			_velocityProfiles[j][1][i] = std::min( sqrt(aLatP2/_curvatureProfile[i]),  _velocityProfiles[j][1][i]);

			// Profil 3 : Sporty
			_velocityProfiles[j][2][i] = std::min( sqrt(aLatP3/_curvatureProfile[i]),  _velocityProfiles[j][2][i]);
		}
	}

	return true;
}

bool velocityProfile::addAccelerationConstraints(double aP1, double aP2, double aP3){

	if(_velocityProfiles.empty()){
		std::cerr<<"addAccelerationConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfile method before."<<std::endl;
        return false;
	}

	for (int j = 0; j < static_cast<int>(_velocityProfiles.size()); ++j)
	{
		for (int i = 0; i < static_cast<int>(_velocityProfiles[j][0].size())-1; ++i)
		{
			// Profil 1 : Defensive 
			_velocityProfiles[j][0][i+1] = std::min(sqrt( pow(_velocityProfiles[j][0][i],2) + 2*aP1*(_distanceProfile[i+1]-_distanceProfile[i]) ),_velocityProfiles[j][0][i+1]);

			// Profil 2 : Normal
			_velocityProfiles[j][1][i+1] = std::min(sqrt( pow(_velocityProfiles[j][1][i],2) + 2*aP2*(_distanceProfile[i+1]-_distanceProfile[i]) ),_velocityProfiles[j][1][i+1]);

			// Profil 3 : Sporty
			_velocityProfiles[j][2][i+1] = std::min(sqrt( pow(_velocityProfiles[j][2][i],2) + 2*aP3*(_distanceProfile[i+1]-_distanceProfile[i]) ),_velocityProfiles[j][2][i+1]);
		}
	}

	return true;
}

bool velocityProfile::addDecelerationConstraints(double gP1, double gP2, double gP3){

	if(_velocityProfiles.empty()){
		std::cerr<<"addDecelerationConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfiles method before."<<std::endl;
        return false;
	}

	for (int j = 0; j < static_cast<int>(_velocityProfiles.size()); ++j)
	{
		for (int i = static_cast<int>(_velocityProfiles[j][0].size())-1; i > 0; --i)
	    {
	        _velocityProfiles[j][0][i-1] = std::min(_velocityProfiles[j][0][i] + gP1*(_distanceProfile[i]-_distanceProfile[i-1]), _velocityProfiles[j][0][i-1]);

	        _velocityProfiles[j][1][i-1] = std::min(_velocityProfiles[j][1][i] + gP2*(_distanceProfile[i]-_distanceProfile[i-1]), _velocityProfiles[j][1][i-1]);

	        _velocityProfiles[j][2][i-1] = std::min(_velocityProfiles[j][2][i] + gP3*(_distanceProfile[i]-_distanceProfile[i-1]), _velocityProfiles[j][2][i-1]);
	    }
	}

	return true;
}

bool velocityProfile::cleanVelocityProfiles(double equalDiffPrecent){

	if(_velocityProfiles.empty()){
		std::cerr<<"cleanVelocityProfiles cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfiles method before."<<std::endl;
        return false;
	}

	if(_velocityProfiles.size() > 1){
		// Vector which contains final case for the velocity profiles. The aim is that each case is unique
		std::vector<std::vector<std::vector<double>>> keepedVP;

		for (int j = 0; j < _velocityProfiles.size()-1; ++j)
		{
			int i = j+1;
			bool vpEquals = false;
			int equalVal = 0;

			// Compare case j to cases j+1 to the end
			while(i < _velocityProfiles.size() && !vpEquals)
			{
				// Compare values at each index (distance)
				for (int k = 0; k < _velocityProfiles[j][0].size(); ++k)
				{
					if(_velocityProfiles[j][0][k] == _velocityProfiles[i][0][k]) equalVal++;
				}
				
				if(equalVal >= static_cast<int>(_velocityProfiles[j][0].size()) * ((100.0-equalDiffPrecent)/100.0)) vpEquals = true;
							
				equalVal = 0;
				i++;
			}

			if(!vpEquals && j == _velocityProfiles.size()-2){
				keepedVP.push_back(_velocityProfiles[j]);
				keepedVP.push_back(_velocityProfiles[j+1]);
			}
			else if(!vpEquals) keepedVP.push_back(_velocityProfiles[j]);
		}

		// Update _velocityProfiles attribute
		_velocityProfiles = keepedVP;
	}

	return true;
}

bool velocityProfile::addIdmPrecedingObjectConstraints(
	double actualGap, double objVelocity, double preObjVelocity, double currentDist,
	double aP1, double aP2, double aP3, double aExpo, double comfDeceleration,
	double d0, double timeGap){

	if(_velocityProfiles.empty()){
		std::cerr<<"addPrecedingObjectConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfiles method before."<<std::endl;
        return false;
	}

	//--------------------------------------------
    // First step : Find idx in profiles from _distanceProfile according to currentDist
	//--------------------------------------------
	if(currentDist > _distanceProfile.back()){
		std::cerr<<"addPrecedingObjectConstraints cannot be used. currentDist is outside of this velocityProfile."<<std::endl;
        return false;
	}

	int workIdx = floor( ( (_distanceProfile.size()-1) * currentDist )/_distanceProfile.back() );

	//--------------------------------------------
    // Second step : 
    // . Implement Intelligent Driver Model (IDM)
    // . Update VP
	//--------------------------------------------

	std::vector<double> aP = std::vector<double>{aP1,aP2,aP3};

	/* For each case in _velocityProfiles */
	for (int idxCase = 0; idxCase < static_cast<int>(_velocityProfiles.size()); ++idxCase)
	{
		/* For each profile */
		for (int idxProfile = 0; idxProfile < static_cast<int>(_velocityProfiles[idxCase].size()); ++idxProfile)
		{
			/* Implement Intelligent Driver Model (IDM) */
			
			/** Desired gap **/
			double desiredGap = d0 + timeGap * objVelocity;
			desiredGap += ( (objVelocity * (objVelocity-preObjVelocity))/(2*sqrt(aP[idxProfile]*comfDeceleration)) );
			
			/** acceleration **/
			double v_point = aP[idxProfile] * (1 - pow(objVelocity/_velocityProfiles[idxCase][idxProfile][workIdx],aExpo) - pow(desiredGap/actualGap,2));

			/** Update VP **/
			_velocityProfiles[idxCase][idxProfile][workIdx] = std::min(objVelocity / pow(1-v_point/aP[idxProfile],1.0/aExpo),_velocityProfiles[idxCase][idxProfile][workIdx]);
		}
	}

	return true;
}

bool velocityProfile::addIdmAccelerationConstraints(double objVelocity, double currentDist, double aP1, double aP2, double aP3, double aExpo){

	if(_velocityProfiles.empty()){
		std::cerr<<"addIdmAccelerationConstraints cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfiles method before."<<std::endl;
        return false;
	}

	//--------------------------------------------
    // First step : Find idx in profiles from _distanceProfile according to currentDist
	//--------------------------------------------
	if(currentDist > _distanceProfile.back()){
		std::cerr<<"addIdmAccelerationConstraints cannot be used. currentDist is outside of this velocityProfile."<<std::endl;
        return false;
	}

	int workIdx = floor( ( (_distanceProfile.size()-1) * currentDist )/_distanceProfile.back() );

	//--------------------------------------------
    // Second step : 
    // . Implement Intelligent Driver Model (IDM)
    // . Update VP
	//--------------------------------------------

	std::vector<double> aP = std::vector<double>{aP1,aP2,aP3};

	/* For each case in _velocityProfiles */
	for (int idxCase = 0; idxCase < static_cast<int>(_velocityProfiles.size()); ++idxCase)
	{
		/* For each profile */
		for (int idxProfile = 0; idxProfile < static_cast<int>(_velocityProfiles[idxCase].size()); ++idxProfile)
		{
			/* Implement Intelligent Driver Model (IDM) */
			
			/** acceleration **/
			double v_point = aP[idxProfile] * (1 - pow(objVelocity/_velocityProfiles[idxCase][idxProfile][workIdx],aExpo));

			/** Update VP **/
			_velocityProfiles[idxCase][idxProfile][workIdx] = std::min(objVelocity / pow(1-v_point/aP[idxProfile],1.0/aExpo),_velocityProfiles[idxCase][idxProfile][workIdx]);
		}
	}

	return true;
}

bool velocityProfile::getVelocity(int desiredCase, int desiredProfile, double atDistance, double &velocity){

	if(_velocityProfiles.empty()){
		std::cerr<<"getVelocity cannot be used. _velocityProfiles must be initialized. Please use setVelocityProfiles method before."<<std::endl;
        return false;
	}

	if(desiredCase > static_cast<int>(_velocityProfiles.size())-1){
		std::cerr<<"getVelocity cannot be used. desiredCase does not exist in _velocityProfiles."<<std::endl;
        return false;
	}

	if(desiredProfile < 0 || desiredProfile > 2){
		std::cerr<<"getVelocity cannot be used. desiredProfile must be in [0..2] <=> defensive (0), normal (1) or sporty (2)."<<std::endl;
        return false;
	}

	if(atDistance < 0 || atDistance > _distanceProfile.back()){
		std::cerr<<"getVelocity cannot be used. atDistance is outside of this velocityProfile."<<std::endl;
        return false;
	}

	// Find idx in profiles from _distanceProfile according to atDistance
	int idx = floor( ( (_distanceProfile.size()-1) * atDistance )/_distanceProfile.back() );

	// Update output
	velocity = _velocityProfiles[desiredCase][desiredProfile][idx];

	return true;
}