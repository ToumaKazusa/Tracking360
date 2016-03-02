#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct direction{
	int xdir;
	int ydir;
};

struct degree{
	float vt;
	float hr;
};

//Smoof a continus array
void Smoof (double *Ar, int Range, int Width) 
{
	int i, j;
	double v, temp = 0;
	for(i = 0;i < Range;i ++)temp += Ar[i];
	for(i = 0;i < Width - Range;i ++)
	{
		v =  temp / Range;
		temp -= Ar[i];
		temp += Ar[i + Range];
		Ar[i] = v;
	}
	j = Range;
	for(;i < Width;i ++)
	{
		v = temp / j;
		j --;
		temp -= Ar[i];
		Ar[i] = v;
	}
	return;
}

//smoof a given mat M
void Smoofimg (Mat M, int Range, int Width, int Height)
{
	int i, j, i1, j1;
	double sum;
	int count;
	Mat temp = M.clone(); 
	for(i = 0;i < Width;i ++)
	{
		for(j = 0;j < Height;j ++)
		{
			sum = 0;
			count = 0;
			for(i1 = std::max(i - Range, 0); i1 < std::min(Width, i + Range); i1 ++)
			{
				for(j1 = std::max(j - Range, 0); i1 < std::min(Height, i + Range); j1 ++)
				{
					count ++;
					sum += temp.at<double>(i1, j1);
				}
			}
			M.at<double>(i, j) = sum/count;
		}
	}
	return;
}

direction Cameramotion (Mat Previmg, Mat Currimg, int Width, int Height){
	double *sump, *sumc, *Pi, *Ci, *diff;
	int i, j;
	direction motion;
	//Calculate x-motion of image 
	sump = new double[Width];
	sumc = new double[Width];
	for(i = 0; i < Width; i++)
	{	
		sump[i] = 0;
		sumc[i] = 0;
		for(j = 0; j < Height; j++)
		{
			sump[i] += Previmg.at<double>(j, i);
			sumc[i] += Currimg.at<double>(j, i);;
		}
	}
	diff = new double[321];
	//Assume moving is within [-160, 160]
	//Smoof the difference and get delay
	Smoof(sump, 10, Width);
	Smoof(sumc, 10, Width);
	for(i = -160;i < 160;i ++)
	{
		diff[i] = 0;
		for(j = 160;j < Width - 160;j ++)
		{
			diff[i + 160] += (sump[j] - sumc[j + i])*(sump[j] - sumc[j + i]);
		}
	}
	double min = diff[0];
	int pos = 0;
	for(i = 1;i < 321;i ++)
	{
		if(diff[i] < min)
		{
			min = diff[i];
			pos = i;
		}
	}
	motion.xdir = pos - 160;
	
	//Calculate y-motion of image 
	delete[] sump;
	delete[] sumc;
	sump = new double[Height];
	sumc = new double[Height];
	for(i = 0; i < Height; i++)
	{	
		Pi = Previmg.ptr<double>(i);
		Ci = Currimg.ptr<double>(i);
		sump[i] = 0;
		sumc[i] = 0;
		for(j = 0; j < Width; j++)
		{
			sump[i] += Pi[j];
			sumc[i] += Ci[j];
		}
	}
	delete[] diff;
	diff = new double[241];
	//Assume moving is within [-120, 120]
	//Smoof the difference and get delay
	Smoof(sump, 10, Height);
	Smoof(sumc, 10, Height);
	for(i = -120;i < 120;i ++)
	{
		diff[i] = 0;
		for(j = 120;j < Width - 120;j ++)
		{
			diff[i + 120] += (sump[j] - sumc[j + i])*(sump[j] - sumc[j + i]);
		}
	}
	min = diff[0];
	pos = 0;
	for(i = 1;i < 241;i ++)
	{
		if(diff[i] < min)
		{
			min = diff[i];
			pos = i;
		}
	}
	motion.ydir = pos - 120;
	delete[] sump;
	delete[] sumc;
	delete[] diff;
	return motion;	
}

degree Cameradegree (Mat Previmg, Mat Currimg, int Height, int Width){
	direction motion = Cameramotion(Previmg, Currimg, Width, Height);
	Smoofimg(Previmg, 5, Width, Height);
	Smoofimg(Currimg, 5, Width, Height);
	int i, j;
	int thershold = 20;
	Mat Binary = Mat::zeros(100, 100, CV_8U);
	for(i = std::max(0, motion.ydir);i < std::min(Height, Height + motion.xdir);i ++)
	{
		for(j = std::max(0, motion.xdir);j < std::min(Width, Width + motion.xdir);j ++)
		{
			if(abs(Currimg.at<double>(i, j) - Previmg.at<double>(i - motion.ydir, j - motion.xdir)) < thershold)
				Binary.at<int>(i, j) = 0;
			else Binary.at<int>(i, j) = 1;	
		}
	}
	//Current function: Get motion vector<x, y> by Cameramotion, then thershold imagine into new mat Binary,
	//in Binary, 1 means this point have a large difference -> indicate moving object; 0 -> static background
	//To be done: Image segmentation
	
	degree result;
	result.vt = 0;
	result.hr = 0;
	return result;
}
