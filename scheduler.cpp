#include <iostream>
#include <pthread.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DEBUG_360 true

using namespace cv;
using namespace std;

class ImgBuffer
{
    enum {
        max_buff_size = 5,
    };
    public:
        bool PushImgBuff(Mat& buff);
        bool PopImgBuff(Mat& buff);
        ImgBuffer(pthread_mutex_t* mutex);
    private:
        ImgBuffer(const ImgBuffer &obj);
        Mat _buff[max_buff_size];
        int head, tail, sz;
        bool debug;
        pthread_mutex_t* _mutex;
};

ImgBuffer::ImgBuffer(pthread_mutex_t* mutex)
{
    head = tail = sz = 0;
    _mutex = mutex;
    debug = DEBUG_360;
}

bool ImgBuffer::PushImgBuff(Mat& buf)
{
    pthread_mutex_lock(_mutex);

    // if buffer is going to be overrided, 
    // throw away old buffer and move tail index by one
    if(sz == max_buff_size)
    {
        if(debug)
            fprintf(stderr, "buffer overflow detected at %d in %s \n", 
                    __LINE__, __FUNCTION__);
        tail++;
    }

    _buff[head++] = buf;
    head  %= max_buff_size;
    sz++;

    pthread_mutex_unlock(_mutex);

    return true;
}

bool ImgBuffer::PopImgBuff(Mat& buf)
{
    if(!sz)
    {
        if(debug)
            fprintf(stderr, "buffer underflow detected at %d in %s \n", 
                    __LINE__, __FUNCTION__);
        return false;
    }

    pthread_mutex_lock(_mutex);
    buf = _buff[tail++];
    tail %= max_buff_size;
    --sz;
    pthread_mutex_unlock(_mutex);

    return true;
}

 int main( int argc, char** argv )
 {
     pthread_mutex_t lock;

     if(pthread_mutex_init(&lock, NULL) != 0)
     {
         fprintf(stderr, "mutex init failed\n");
         return -1;
     }


     ImgBuffer buf(&lock);
     VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

//    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

//    int iLowH = 0;
//    int iHighH = 179;
//
//    int iLowS = 0; 
//    int iHighS = 255;
//
//    int iLowV = 0;
//    int iHighV = 255;
//
//    //Create trackbars in "Control" window
//    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
//
//    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
//
//    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    int i = 0;
    while (true && i < 20)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        Mat imgHSV;
        //Convert the captured frame from BGR to HSV
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 

        buf.PushImgBuff(imgHSV);
        buf.PopImgBuff(imgHSV);
        char img_name[50];
        sprintf(img_name, "img%d.jpg", i);
        if(false == imwrite(img_name, imgHSV))
        {
            fprintf(stderr, "image write failed\n");
        }
        printf("%s written\n", img_name);
        ++i;
    }

     
        //Mat imgThresholded;

        //inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        //  
        ////morphological opening (remove small objects from the foreground)
        //erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        //dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        ////morphological closing (fill small holes in the foreground)
        //dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        //erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //imshow("Thresholded Image", imgThresholded); //show the thresholded image
        //imshow("Original", imgOriginal); //show the original image

        //if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        //{
        //    cout << "esc key is pressed by user" << endl;
        //    break; 
        //}
   return 0;
}
