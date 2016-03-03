#include <stdio.h>
#include "coordinates.h"

using namespace cv;
using namespace std;

struct direction{
	int xdir;
	int ydir;
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
				for(j1 = std::max(j - Range, 0); j1 < std::min(Height, j + Range); j1 ++)
				{
					count ++;
					sum += temp.at<uchar>(j1, i1);
				}
			}
			M.at<uchar>(j, i) = sum/count;
		}
	}
	return;
}

direction Cameramotion (Mat& Previmg, Mat& Currimg, int Width, int Height){
	double*sump, *sumc, *diff;
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
			sump[i] += Previmg.at<uchar>(j, i);
			sumc[i] += Currimg.at<uchar>(j, i);
		}
		sump[i] /= Height;
		sumc[i] /= Height;
	}
	diff = new double[321];
	//Assume moving is within [-160, 160]
	//Smoof the difference and get delay
	Smoof(sump, 10, Width);
	Smoof(sumc, 10, Width);
	for(i = -160;i <= 160;i ++)
	{
		diff[i + 160] = 0;
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
		sump[i] = 0;
		sumc[i] = 0;
		for(j = 0; j < Width; j++)
		{
			sump[i] += Previmg.at<uchar>(i, j);
			sumc[i] += Currimg.at<uchar>(i, j);
		}
		sump[i] /= Width;
		sumc[i] /= Width;
	}
	//delete[] diff;
	diff = new double[241];
	//Assume moving is within [-120, 120]
	//Smoof the difference and get delay
	Smoof(sump, 10, Height);
	Smoof(sumc, 10, Height);
	for(i = -120;i <= 120;i ++)
	{
		diff[i + 120] = 0;
		for(j = 120;j < Height - 120;j ++)
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

void Cameradegree (Mat& Previmg, Mat& Currimg, 
        struct degree& result, int Width, int Height)
{
#if 1
	direction motion = Cameramotion(Previmg, Currimg, Width, Height);
#endif
    //cout << "Pass motion detect!" << motion.xdir << ' ' << motion.ydir << endl;
//	Smoofimg(Previmg, 5, Width, Height);
//	Smoofimg(Currimg, 5, Width, Height);
	int i, j;
	int thershold = 30;
	Mat Binary = Mat::zeros(Height, Width, CV_8U);
    //if(false == imwrite("opencvimg1.jpg", Previmg))
    //{
    //    cout << "not saved prev img" << endl;
    //}
    //else
    //{
    //    cout << "saved prev img" << endl;
    //}
#if 1
    static int z = 0;
    char name[50];
    sprintf(name, "img%d.jpg", z++);
    if(false == imwrite(name, Currimg))
    {
        cout << "not saved cur img" << endl;
    }
    //else
    //{
    //    cout << "saved cur img" << endl;
    //}
#endif
    for(i = std::max(0, motion.ydir);i < std::min(Height, Height + motion.ydir);i ++)
	{
		for(j = std::max(0, motion.xdir);j < std::min(Width, Width + motion.xdir);j ++)
		{
			if(abs(Currimg.at<uchar>(i, j) - Previmg.at<uchar>(i - motion.ydir, j - motion.xdir)) < thershold)
				Binary.at<uchar>(i, j) = 0;
			else Binary.at<uchar>(i, j) = 255; //Currimg.at<uchar>(i, j) - Previmg.at<uchar>(i - motion.ydir, j - motion.xdir);	
		}
	}
	//Current function: Get motion vector<x, y> by Cameramotion, then thershold imagine into new mat Binary,
	//in Binary, 1 means this point have a large difference -> indicate moving object; 0 -> static background
	//To be done: Image segmentation
	int w = Width / 32;
	int h = Height / 24;
	int i1, j1, count, si = 0, sj = 0, area = 0;
	result.x = 0;
	result.y = 0;
	for(i = 0;i < h;i ++)
	{
		for(j = 0;j < w;j ++)
		{
			count = 0;
			for(i1 = 0;i1 < 24;i1 ++)for(j1 = 0;j1 < 32;j1 ++)if(Binary.at<uchar>(24 * i + i1, 32 * j + j1) > 0)count ++;
			if(count > 150)
			{
				result.x += j;
				result.y += i;j
				area++;
				sj += j;
				si += i;
			}
		}
	}
	if(area > 100) 
	{
		result.x = result.y = 0;
		return;
	}
	if(s > 0)result.x = 54 * result.x / (sj * w) - 27;
    	else result.x = 0;
	if(s > 0)result.y = 40 * result.y / (si * h) - 20;
	else result.y = 0;
	cout << "Result:" << result.x << " "  << result.y << endl;
}

void updatedegree(struct degree& currentdeg, struct degree& cmd)
{
	if(cmd.x > 0)cmd.x = std::min(180, currentdeg.x + cmd.x);
	else cmd.x = std::max(0, currentdeg.x + cmd.x);
    if(cmd.y < 0)cmd.y = std::min(180, currentdeg.y - cmd.y);
	else cmd.y = std::max(0, currentdeg.y - cmd.y);
}

