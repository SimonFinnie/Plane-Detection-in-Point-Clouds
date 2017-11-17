# Plane-Detection-in-Point-Clouds
This is a method of detecting the equation for n planes in a point cloud.
This was developed as part of a University  assignment. My contributions are solely in the plane finder class. The other classes, which were supplied by the university, are used to format the point cloud into a computationally convenient format.
My contributions were either developed based on known mathematical theory, or developed using my own mathematical intuition. This method is a statistical plane detection method, which uses the statistical RANSAC method to find the best planes in the a point cloud it possibly can.

To use this class, you must have access to c++11 and the eigen libraries. Then you need to update the CMakeList.txt file to include the directory of the eigen library. Then use CMake to build the program.
The format to call this method is ./planefinder input_pointcloud_file.ply output_pointcloud_file.ply planeSizeThreshold percentageSizeOfBiggestPlane probOfFailure
where planeSizeThreshold is the number of points allowed in the smallest plane, percentageSizeOfBiggestPlane is the estimate of the percentage of points which make up the largest plane, and probOfFailure is the probabilty of the method failing to find an accurate plane.

The current implimentation has an error in the least squares method, and so does not use it. Also the automated normal to plane threshold calculation method is less effective in more complex scenes.
