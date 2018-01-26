#include <opencv2/core/core.hpp>
using namespace cv;
int main()
{
    FileStorage fin("../calibration/test.yml",FileStorage::WRITE);
    Mat R = Mat_<uchar>::eye(3,3);
    fin<<"R"<<R;
    fin.release();
    return 1;
}