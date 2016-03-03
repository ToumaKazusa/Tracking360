#ifndef H_COORDINATES_H
#define  H_COORDINATES_H 
#include <opencv2/opencv.hpp>
using namespace cv;

struct degree{
	int x;
	int y;
};

void Cameradegree (Mat& Previmg, Mat& Currimg, struct degree& result, 
        int Width = 640, int Height = 480);

void updatedegree(struct degree& currentdeg, struct degree& cmd);
#endif
