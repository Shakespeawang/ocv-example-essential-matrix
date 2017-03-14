# ocv-example-essential-matrix
This is a simple example of using OpenCV findEssentialMat() and recoverPose() to determine the camera pose using two views.

findEssentialMat() takes feature points from one image, and the corresponding points of where those features have moved to in the second image, and computes the Essential Matrix.

recoverPose() takes the Essential Matrix and the same two sets of points and computes the rotation matrix and the translation between the world of image 1 and the world of image 2.  From that, you get the movement of the camera between image 1 and image 2.

A typical usage of these functions is to use apply them to the result of feature matching or tracking on a pair of images in order to compute the trajectory of the camera between the two images.

Rather than using actual images, I wrote this trivial program to demonstrate what these do using a contrived scene.  For someone learning opencv, the benefit of doing it this way is that you can more easily control the inputs to these function and compare the computed rotation and translation against your expected value.

The contrived scene consists of the corners of some squares and a few points scattered here and there in a 3D-world in front of the original position of the camera, assumed to be at the origin.

Then the camera image is generated from the 3D-world position of the feature points by projecting them on to the image plane.  Then, we apply a rotation and translation to simulate the camera motion, and then project them on the image plane to generate the second image (ie. after the camera has moved).

Next, we can apply findEssentialMat() and recoverPose() to attempt to recover the rotation and translation based only on the projected image points in the original and second image.
