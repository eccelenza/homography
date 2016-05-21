#include <opencv2/opencv.hpp>

#ifdef _MSC_VER
# if CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION == 0 && CV_SUBMINOR_VERSION == 0
// OpenCV 3.0.0
#  if defined(_DEBUG)
#  pragma comment(lib, "opencv_core300d.lib")
#  pragma comment(lib, "opencv_highgui300d.lib")
#  pragma comment(lib, "opencv_hal300d.lib")
#  pragma comment(lib, "opencv_imgproc300d.lib")
#  pragma comment(lib, "opencv_imgcodecs300d.lib")
#  pragma comment(lib, "opencv_video300d.lib")
#  pragma comment(lib, "opencv_ml300d.lib")
#  pragma comment(lib, "opencv_calib3d300d.lib")
#  pragma comment(lib, "opencv_objdetect300d.lib")
#  pragma comment(lib, "opencv_features2d300d.lib")
#  pragma comment(lib, "opencv_photo300d.lib")
#  pragma comment(lib, "opencv_shape300d.lib")
#  pragma comment(lib, "opencv_stitching300d.lib")
#  pragma comment(lib, "opencv_flann300d.lib")
#  pragma comment(lib, "opencv_superres300d.lib")
#  pragma comment(lib, "opencv_ts300d.lib")
#  pragma comment(lib, "opencv_videoio300d.lib")
#  pragma comment(lib, "opencv_videostab300d.lib")
# else
#  pragma comment(lib, "opencv_core300.lib")
#  pragma comment(lib, "opencv_highgui300.lib")
#  pragma comment(lib, "opencv_hal300.lib")
#  pragma comment(lib, "opencv_imgproc300.lib")
#  pragma comment(lib, "opencv_imgcodecs300.lib")
#  pragma comment(lib, "opencv_video300.lib")
#  pragma comment(lib, "opencv_ml300.lib")
#  pragma comment(lib, "opencv_calib3d300.lib")
#  pragma comment(lib, "opencv_objdetect300.lib")
#  pragma comment(lib, "opencv_features2d300.lib")
#  pragma comment(lib, "opencv_photo300.lib")
#  pragma comment(lib, "opencv_shape300.lib")
#  pragma comment(lib, "opencv_stitching300.lib")
#  pragma comment(lib, "opencv_flann300.lib")
#  pragma comment(lib, "opencv_superres300.lib")
#  pragma comment(lib, "opencv_ts300.lib")
#  pragma comment(lib, "opencv_videoio300.lib")
#  pragma comment(lib, "opencv_videostab300.lib")
# endif //#  if defined(_DEBUG)

# endif //# if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION == 3
#endif //#ifdef _MSC_VER