/**
 * @file   label.cpp
 * @author DUFOUR Julien
 * @date   April, 2016
 * @brief  File containing useful methods to manipulate and use the labels given by kitti datasets.
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

#include <label/label.h>

label::label(std::string label_path){
    dlib::deserialize( (label_path + "/decision_function_svm/df.dat").c_str() ) >> _df;
}

// Get the data from kitti´s label files
// And sort them for dlib (svm library -- see : include/dlib-18.18)
void label::generate_svm_data(std::vector<label::sample_type>& samples, std::vector<double>& labels, const std::string path){

	// Set a Map to link each Class with a number
	std::map<std::string, double> idLabels = {{"Car", 1}, {"Cyclist", 2}, {"Pedestrian", 3},
													{"Van", 4}, {"Truck", 5}, {"Tram", 6},
													{"Person_sitting", 7}, {"Misc", 8}};

	label::sample_type m;
	std::string line;
	std::string val;

	// Get the name of labels files
	std::vector<std::string> filenames;
	if(utils::getFileNames(path,"txt",filenames))
	{
		// For each file
		for(auto const& filename: filenames) 
		{
			std::ifstream myfile((path + filename + ".txt").c_str() );
			if(myfile.is_open())
			{
				// For each line
				while(std::getline(myfile,line)){
					std::stringstream ss;
		            ss << line;
		            
		            // Label
		            // Looking for element 3 <=> jump of 2 elements
/*                    for (int i = 0; i < 2; ++i)
                    {
                        ss >> val;
                    }*/
                    ss >> val;
		            if(val.compare("DontCare")!=0)
		            {
		            	labels.push_back(idLabels[val]);

		            	// Alpha <-> Observation angle of object, ranging [-pi..pi]
		            	// Looking for element 6 <=> jump of 2 elements
				        for (int i = 0; i < 2; ++i)
				        {
				        	ss >> val;
				        }
			            ss >> val;
			            m(6) = static_cast<double>(atof(val.c_str()));

			            // Dimensions <-> 3D object dimensions: height, width, length (in meters)
			            // Looking for elements 11 to 13 <=> jump of 4 elements
			            for (int i = 0; i < 4; ++i)
				        {
				        	ss >> val;
				        }
			            ss >> val;
			            m(0) = static_cast<double>(atof(val.c_str()));
			            ss >> val;
			            m(1) = static_cast<double>(atof(val.c_str()));
			            ss >> val;
			            m(2) = static_cast<double>(atof(val.c_str()));

			            // Location <-> 3D object location x,y,z in camera coordinates (in meters)
			            ss >> val;
			            m(3) = static_cast<double>(atof(val.c_str()));
			            ss >> val;
			            m(4) = static_cast<double>(atof(val.c_str()));
			            ss >> val;
			            m(5) = static_cast<double>(atof(val.c_str()));

			            // Add this sample to our set of training samples
			            samples.push_back(m);
		            } 
				}
			}

		}
	}
};

void label::generate_df_function(const std::string label_path){
	try
    {
        std::vector<label::sample_type> samples;
        std::vector<double> labels;

        // First, get our labeled set of training data
        label::generate_svm_data(samples, labels, label_path + "data/");

        std::cout << "There are "<< samples.size() << " samples." << std::endl;

        // The main object in this example program is the one_vs_one_trainer.  It is essentially 
        // a container class for regular binary classifier trainer objects.  In particular, it 
        // uses the any_trainer object to store any kind of trainer object that implements a 
        // .train(samples,labels) function which returns some kind of learned decision function.  
        // It uses these binary classifiers to construct a voting multiclass classifier.  If 
        // there are N classes then it trains N*(N-1)/2 binary classifiers, one for each pair of 
        // labels, which then vote on the label of a sample.
        //
        // In this example program we will work with a one_vs_one_trainer object which stores any 
        // kind of trainer that uses our label::sample_type samples.
        //typedef dlib::one_vs_one_trainer<dlib::any_trainer<label::sample_type>> ovo_trainer;


        // Finally, make the one_vs_one_trainer.
        label::ovo_trainer trainer;


        // Next, we will make two different binary classification trainer objects.  One
        // which uses kernel ridge regression and RBF kernels and another which uses a
        // support vector machine and polynomial kernels.  The particular details don't matter.
        // The point of this part of the example is that you can use any kind of trainer object
        // with the one_vs_one_trainer.
        //typedef dlib::polynomial_kernel<label::sample_type> poly_kernel;
        //typedef dlib::radial_basis_kernel<label::sample_type> rbf_kernel;

        // make the binary trainers and set some parameters
        dlib::krr_trainer<label::rbf_kernel> rbf_trainer;
        dlib::svm_nu_trainer<label::poly_kernel> poly_trainer;
        
        // Note to understand Gamma parameter : http://stackoverflow.com/questions/4629505/svm-hard-or-soft-margins
        poly_trainer.set_kernel(label::poly_kernel(0.1, 1, 4));
        rbf_trainer.set_kernel(label::rbf_kernel(0.1));

        // Now tell the one_vs_one_trainer that, by default, it should use the rbf_trainer
        // to solve the individual binary classification subproblems.
        trainer.set_trainer(rbf_trainer);
        // We can also get more specific.  Here we tell the one_vs_one_trainer to use the
        // poly_trainer to solve the class 1 vs class 2 subproblem.  All the others will
        // still be solved with the rbf_trainer.
        trainer.set_trainer(poly_trainer, 1, 2);
        trainer.set_trainer(poly_trainer, 2, 3);
        trainer.set_trainer(poly_trainer, 1, 3);

        // Now let's do 5-fold cross-validation using the one_vs_one_trainer we just setup.
        // As an aside, always shuffle the order of the samples before doing cross validation.  
        // For a discussion of why this is a good idea see the svm_ex.cpp example.
        dlib::randomize_samples(samples, labels);
        std::cout << "cross validation: \n" << cross_validate_multiclass_trainer(trainer, samples, labels, 5) << std::endl;
        // The output is shown below.  It is the confusion matrix which describes the results.  Each row 
        // corresponds to a class of data and each column to a prediction.  Reading from top to bottom, 
        // the rows correspond to the class labels if the labels have been listed in sorted order.  So the
        // top row corresponds to class 1, the middle row to class 2, and the bottom row to class 3.  The
        // columns are organized similarly, with the left most column showing how many samples were predicted
        // as members of class 1.
        // 
        // So in the results below we can see that, for the class 1 samples, 60 of them were correctly predicted
        // to be members of class 1 and 0 were incorrectly classified.  Similarly, the other two classes of data
        // are perfectly classified.
        /*
            cross validation: 
            60  0  0 
            0 70  0 
            0  0 80 
        */

        // Next, if you wanted to obtain the decision rule learned by a one_vs_one_trainer you 
        // would store it into a one_vs_one_decision_function.
        dlib::one_vs_one_decision_function<label::ovo_trainer> df = trainer.train(samples, labels);

        std::cout << "predicted label: "<< df(samples[0])  << ", true label: "<< labels[0] << std::endl;
        std::cout << "predicted label: "<< df(samples[90]) << ", true label: "<< labels[90] << std::endl;
        // The output is:
        /*
            predicted label: 2, true label: 2
            predicted label: 1, true label: 1
        */


        // If you want to save a one_vs_one_decision_function to disk, you can do
        // so.  However, you must declare what kind of decision functions it contains. 
        /*dlib::one_vs_one_decision_function<ovo_trainer,
        dlib::decision_function<poly_kernel>,  // This is the output of the poly_trainer
        dlib::decision_function<rbf_kernel>    // This is the output of the rbf_trainer
        > df1,df2;*/
        label::decision_function df1,df2;


        // Put df into df2 and then save df2 to disk.  Note that we could have also said
        // df2 = trainer.train(samples, labels);  But doing it this way avoids retraining.
        df1 = df;
        dlib::serialize( (label_path + "/decision_function_svm/df.dat").c_str() ) << df1;

        // load the function back in from disk and store it in df3.  
        dlib::deserialize( (label_path + "/decision_function_svm/df.dat").c_str() ) >> df2;


        // Test df2 to see that this worked.
        std::cout << std::endl;
        std::cout << "predicted label: "<< df2(samples[0])  << ", true label: "<< labels[0] << std::endl;
        std::cout << "predicted label: "<< df2(samples[90]) << ", true label: "<< labels[90] << std::endl;
        // Test df3 on the samples and labels and print the confusion matrix.
        std::cout << "test deserialized function: \n" << dlib::test_multiclass_decision_function(df2, samples, labels) << std::endl;

        // Finally, if you want to get the binary classifiers from inside a multiclass decision
        // function you can do it by calling get_binary_decision_functions() like so:
        dlib::one_vs_one_decision_function<label::ovo_trainer>::binary_function_table functs;
        functs = df.get_binary_decision_functions();
        std::cout << "number of binary decision functions in df: " << functs.size() << std::endl;

    }
    catch (std::exception& e)
    {
        std::cout << "exception thrown!" << std::endl;
        std::cout << e.what() << std::endl;
    }
}


int label::classObb(utils::OBBcube obb, utils::transformMatrix tf_mat){

    label::sample_type m;

    m(0) = obb.z;

    if(obb.x > obb.y){
        m(1) = obb.y;
        m(2) = obb.x;
    }
    else
    {
        m(1) = obb.x;
        m(2) = obb.y;
    }
    
    Eigen::MatrixXd obbPos;

    obbPos.resize(4,1);
    // Object CS is set on the bottom on labelled files.
    // See : "cs_overview" in datasets/label
    obbPos(0,0) = static_cast<double>(obb.position(0));
    obbPos(1,0) = static_cast<double>(obb.position(1));
    obbPos(2,0) = static_cast<double>(obb.position(2))-obb.z/2.0;
    obbPos(3,0) = 1.0;

    obbPos = (
            tf_mat.calib_cam_to_camRect
            * tf_mat.calib_velo_to_cam
            * tf_mat.calib_imu_to_velo
            * tf_mat.oxts_pose.inverse()
            * obbPos);

    m(3) = obbPos(0,0);
    m(4) = obbPos(1,0);
    m(5) = obbPos(2,0);

    // Object Observation Angle.
    // See : "cs_overview" in datasets/label
    double beta = atan2(obbPos(2,0),obbPos(0,0));
    Eigen::Vector3f ea = obb.quat.toRotationMatrix().eulerAngles(0, 1, 2); 
    double rY = ea[2];

    m(6) = -rY + beta;
    
    return _df(m);
}