/*
 *  c++ implementation of Lowess weighted regression by 
 *  Peter Glaus http://www.cs.man.ac.uk/~glausp/
 *
 *
 *  Based on fortran code by Cleveland downloaded from:
 *  http://netlib.org/go/lowess.f
 *  original author:
* wsc@research.bell-labs.com Mon Dec 30 16:55 EST 1985
* W. S. Cleveland
* Bell Laboratories
* Murray Hill NJ 07974
 *  
 *  See original documentation in the .cpp file for details.
 * 
 */
#ifndef LOWESS_H
#define LOWESS_H

#include<vector>

void lowess(const std::vector<double> &x, const std::vector<double> &y, double f, long nsteps, double delta, std::vector<double> &ys, std::vector<double> &rw, std::vector<double> &res);

void lowess(const std::vector<double> &x, const std::vector<double> &y, double f, long nsteps, std::vector<double> &ys);

void lowest(const std::vector<double> &x, const std::vector<double> &y, double xs, double &ys, long nleft, long nright, std::vector<double> &w,bool userw,  std::vector<double> &rw, bool &ok);

#endif
