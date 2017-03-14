#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// controls if you want the projection co-ordinates
// of the transformed scene to be randomly perturbed so that
// you can observe the effect
static bool want_random_perturb = false;

static double width = 1000;
static double height = 1000;
static double fov = 60; // field of view in degrees
static double tan_half_fov = tan(fov * CV_PI / 180.0);
static double f_in_pixels = width / tan_half_fov;

Mat K = (Mat_<double>(3,3) <<
    f_in_pixels, 0, (width-1)/2.0,
    0, f_in_pixels, (height-1)/2.0,
    0, 0, 1);

// Rotation matrix to euler angles
Vec3d to_euler_angles(Mat &R) {
  Mat mtxR, mtxQ, Rx, Ry, Rz;
  Vec3d euler_angles = RQDecomp3x3(R, mtxR, mtxQ, Rx, Ry, Rz);
  return euler_angles;
}

// Euler angles to rotation matrix (homogeneous)
Mat to_rotation_matrix(Vec3d& v) {
  Mat Rx = (Mat_<double>(3,3) <<
      1, 0, 0,
      0, cos(v[0]), -sin(v[0]),
      0, sin(v[0]), cos(v[0]));
  Mat Ry = (Mat_<double>(3,3) <<
      cos(v[1]), 0, sin(v[1]),
      0, 1, 0,
      -sin(v[1]), 0, cos(v[1]));
  Mat Rz = (Mat_<double>(3,3) <<
      cos(v[2]), -sin(v[2]), 0,
      sin(v[2]), cos(v[2]), 0,
      0, 0, 1);

  Mat R = Rx * Ry * Rz;
  return R;
}

// Convert Mat to vector of Point2d
// I - projected points
// pts - output vector of Point2d
void mat_to_p2d(const Mat& I, vector<Point2d>& pts) {
  for(int i = 0; i < I.rows; i++) {
    Point2d pt(I.at<double>(i, 0), I.at<double>(i, 1));
    pts.push_back(pt);
  }
}

Mat calc_essential_mat(Mat& I1, Mat& I2, Mat& K) {
  vector<Point2d> pts1, pts2;
  mat_to_p2d(I1, pts1);
  mat_to_p2d(I2, pts2);

  Mat mask;
  Mat E = findEssentialMat(pts1, pts2, K);
  return E;
}

void recover_pose(Mat& E, Mat& I1, Mat& I2, Mat& K, Mat& R, Mat& t) {
  vector<Point2d> pts1, pts2, mask;
  mat_to_p2d(I1, pts1);
  mat_to_p2d(I2, pts2);

  recoverPose(E, pts1, pts2, K, R, t);
}

void show_image_camera(string s, Mat &I) {
  Mat im = Mat::zeros(width, height, CV_8UC1);

  for(int i = 0; i < I.rows; i++) {
    int x = I.at<double>(i, 0);
    int y = I.at<double>(i, 1);

    if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
      circle(im, Point(x,y), 3, Scalar(255));
    } else {
      cout << "Point out of image: " << x << "," << y << endl;
    }
  }

  string win_name = "Projected Image " + s;
  namedWindow(win_name);
  imshow(win_name, im);
  waitKey(0);
}

// Rotate Q1, by the given euler angles, return in Q2
void euler_to_rodrigues(Vec3d& euler, Mat& Q1) {
  Mat R = to_rotation_matrix(euler);

  Rodrigues(R, Q1);
}

void perturb(Mat& I, double amt = 0.05) {
  float r = amt * (-0.5 + static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
  for(int i = 0; i < I.rows; i++) {
    I.at<double>(i,0) = I.at<double>(i,0) + r;
    I.at<double>(i,1) = I.at<double>(i,1) + r;
  }
}

int main(int argc, char **argv) {

  // Sample points in world, homegenous coordinates
  //
  // 25x25 corners of square at distance 100,
  // 25x25 corners of square at distance 125,
  // 25x25 corners of square at distance 175
  // Point at distance 30
  // Point at distance 20
  // Point at distance 15

  Mat Q = (Mat_<double>(15,3) <<
    -50, -50, 100,
    -25, -50, 100,
    -25, -25, 100,
    -50, -25, 100,
     50,  50, 125,
     25,  50, 125,
     25,  25, 125,
     50,  25, 125,
    -50,  50, 175,
    -25,  50, 175,
    -25,  25, 175,
    -50,  25, 172,
      3,  -3, 300,
      3,  -3, 100,
      3,  -3, 20);

  Mat I1;
  Mat distCoeffs;
  projectPoints(Q, Vec3d(0,0,0), Vec3d(0,0,0), K, distCoeffs, I1);

  cout << "Displaying the original scene" << endl;
  show_image_camera("Original", I1);

  Vec3d euler_degrees(-8.0, 4.0, 0.0);
  cout << "Rotating scene by these euler angles (degrees): " << endl << euler_degrees << endl;
  Mat rod;
  Vec3d euler = euler_degrees * (CV_PI / 180.0);
  euler_to_rodrigues(euler, rod);

  Vec3d translation(0, -10, 0);
  cout << "Translating scene by: " << endl << translation << endl;

  Mat I2;
  Mat distCoeffs2;
  projectPoints(Q, rod, translation, K, distCoeffs2, I2);

  if (want_random_perturb) {

    // Randomly perturn the projected points +/- some random value
    // so that you can see what effect it has on the computation of
    // the essential matrix and recovery of pose
    perturb(I2, 5.0);
  }

  cout << "Displaying the rotated and translated scene" << endl;
  show_image_camera("Transformed", I2);

  cout << "Attempting to compute the essential matrix and recover the pose using the image points from the original and the transformed image" << endl;

  Mat E = calc_essential_mat(I1, I2, K);
  cout << "Essential Matrix:" << endl << E << endl;

  Mat R, t;
  recover_pose(E, I1, I2, K, R, t);

  Vec3d recovered_euler = to_euler_angles(R);
  cout << "Rotation Matrix:" << endl << R << endl;

  cout << "Euler Angles: " << endl << recovered_euler << endl;

  cout << "Translation - notice only correct up to a scaling factor:" << endl << t << endl;
}
