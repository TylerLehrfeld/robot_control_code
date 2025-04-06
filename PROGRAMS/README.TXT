dataReaderHelpers.h: a set of functions that helps main read and write data
distortionCorrection.h: a module that defines a distortion correction polynomial
helperFunctions.h: a set of helper functions used throughout the program
main.cpp: execution file that creates output files and comparisons
mainHelper.h: a helper function file that splits up the tasks of main
Makefile: a way to easily make main or test executables
Matrix-test.h: a module that tests the matrix class
Matrix.cpp: a module that implements the matrix class
Matrix.h: a module that defines the matrix class
Pivot-test.h: defines testPivot function which random pivots and then checks if the algorithm can make sense of them
Pivot.cpp: implementation for Pivot class
Pivot.h: define the Pivot class
PointCloudGenerator.cpp: generates a random point cloud for testing
PointCloudGenerator.h: defines the pointCloudGenerator class
PointCloudTest.h: definines test for point cloud registration
PointCloudTransform.h: implements computing a point cloud registration with SVD
test.cpp: runs all of the tests if given arguments: matrix transform pivot point-cloud
Transform-test.h: defines tests for Frame transformations
Transform.cpp: implements frame Transformation functionality
Transform.h: definies the transform class

main: executable
test: executable (with parameters, see instructions)

Eigen/Dense: in dependencies, there is a folder called eigen-3.4.0. It contains a library that I use for SVD. Here is the link: https://eigen.tuxfamily.org/dox/GettingStarted.html
