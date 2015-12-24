static const int nodes = 5; // must be number of elements in points + 1
static const int cross_validation = 10; // how many validation tests
static const int leave_n_out = 10; // number of elements to leave out
// seed for cross_validation,
// should be about {number_of_data} / {leave_n_out}
static const int validation_seed = 10;

// define how to normalize data structure
/*
  @define NORMALIZE_DATA
  tmp.x /= 1000.0;
  tmp.y += 300.0;
  tmp.y /= 1000.0;
  tmp.z /= 400.0;
  tmp.neck_y += 45.0;
  tmp.neck_y /= 90.0;
*/

// input features
struct points
{
  double x;
  double y;
  double z;
  double neck_y;
};

// note : to change other learning parameters, revise SetParameters in
// aero_startup/.templates/aero_object_manipulation/calibration/AeroCreateHandCalbration.cc
