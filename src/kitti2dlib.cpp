#include <ros/package.h>
#include <label/label.h>

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "velodyne_tracking");

    // Create PATHS to files from ros package path.
    string label_path = ros::package::getPath("velodyne_tracking") + "/datasets/label/";

    // Generate decision function (SVM)
    label::generate_df_function(label_path);

    return 0;
}


