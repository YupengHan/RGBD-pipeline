#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

void rgb_window(bool* mask_img, int* scores, int* uvl);

void find_loc(float* pts, int ptnum, int* scores, float* xyz_limits, float* device_pred_xyz);
void find_loc_fine(float* pts, int ptnum, int* scores, float* xyz_limits, float* device_pred_xyz);
