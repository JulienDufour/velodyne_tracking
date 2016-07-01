/**
 * @file   label.h
 * @author DUFOUR Julien
 * @date   April, 2016
 * @brief  File containing useful headers to manipulate and use the labels given by kitti datasets.
 *
 * To use the labels, the aim is to create and use an Support Vector Machines (SVM) model thanks to the dlib library.
 * Here, an One-Vs-One (ovo) trainer is used.
 *
 * The labels are :
 * _ 1 for "Car"
 * _ 2 for "Cyclist"
 * _ 3 for "Pedestrian"
 * _ 4 for "Van"
 * _ 5 for "Truck"
 * _ 6 for "Tram"
 * _ 7 for "Person_sitting"
 * _ 8 for "Misc"
 *
 * The criterias are :
 * _ Observation angle of object
 * _ 3D object dimensions: height, width, length (in meters)
 * _ 3D object location x,y,z in camera coordinates
 *
 * @see include/dlib-18.18
 */

#include <string>
#include <map>
#include <vector>

#include <utils/dataio.h>

// SVM Lib
#include <dlib/svm_threaded.h>
#include <dlib/rand.h>

#ifndef LABEL_H
#define LABEL_H

class label{

    // For SVM Classification 
    // Our data will be 7-dimensional data. 
    // dimensions -- location -- alpha. For more informations see : datasets/label/readme.txt
    // So declare an appropriate type to contain these points.
    typedef dlib::matrix<double,7,1> sample_type;

    typedef dlib::one_vs_one_trainer<dlib::any_trainer<sample_type> > ovo_trainer;
    typedef dlib::radial_basis_kernel<sample_type> rbf_kernel;
    typedef dlib::polynomial_kernel<sample_type> poly_kernel;
    typedef dlib::one_vs_one_decision_function<ovo_trainer, dlib::decision_function<poly_kernel>, dlib::decision_function<rbf_kernel>> decision_function;

    decision_function _df;

private:
	label();

    /**
     *  @brief generate_svm_data Get data from from kitti´s label files and sort them for dlib (svm library -- see : include/dlib-18.18).
     *  @param samples           Variables whiches represent an object.         (output)
     *  @param labels            Correspondant label for each sample.           (output)
     *  @param path              Path to directory which contains label files.  (input)
     */
	static void generate_svm_data(std::vector<sample_type>& samples, std::vector<double>& labels, const std::string path);

public:
    /**
     *  @brief Constructor Load a decision function which was saved into "datasets/label/decision_function_svm".
     *  @param label_path           Path to directory label in datasets.  (input)
     */
    label(std::string label_path);

	/**
     *  @brief generate_df_function Set an one_vs_one_trainer and save it into "datasets/label/decision_function_svm".
     *  @param label_path           Path to directory label in datasets.  (input)
     */
	static void generate_df_function(const std::string label_path);

    /**
     *  @brief classObb Use SVM decision function to determine the type of seen object, represented by an OBBcube. 
     *  @param obb      Represent an object (size, pos and quat) in global CS system (first pose of the car in Mercator CS).  (input)
     *  @param tf_mat   Struct witch contains all transformation matrix.  (input)
     *  @return         {{"Car", 1}, {"Cyclist", 2}, {"Pedestrian", 3}, {"Van", 4}, {"Truck", 5}, {"Tram", 6}, {"Person_sitting", 7}, {"Misc", 8}}.
     */
    int classObb(utils::OBBcube obb, utils::transformMatrix tf_mat);

};

#endif