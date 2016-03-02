#include <iostream>
#include <pthread.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DEBUG_360 true

using namespace cv;
using namespace std;

bool ProgramExit = false;

class ImgBuf
{
    enum {
        max_buff_size = 5,
    };
    public:
        bool PushImgBuff(Mat& buff);
        bool PopImgBuff(Mat& buff);
        void SetMutex(pthread_mutex_t* mutex);
        ImgBuf(pthread_mutex_t* mutex);
        ImgBuf();
    private:
        ImgBuf(const ImgBuf &obj);
        Mat _buff[max_buff_size];
        int head, tail, sz;
        bool debug;
        pthread_mutex_t* _mutex;
};

ImgBuf::ImgBuf()
{
    head = tail = sz = 0;
    _mutex = NULL;
    debug = DEBUG_360;
}

ImgBuf::ImgBuf(pthread_mutex_t* mutex)
{
    head = tail = sz = 0;
    _mutex = mutex;
    debug = DEBUG_360;
}

void ImgBuf::SetMutex(pthread_mutex_t* mutex)
{
    _mutex = mutex;
}

bool ImgBuf::PushImgBuff(Mat& buf)
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

bool ImgBuf::PopImgBuff(Mat& buf)
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

class ImgProducer
{
    public:
        ImgProducer(ImgBuf*);
        bool Produce(void);
    private:
        ImgProducer(const ImgBuf &obj);
        ImgBuf* ImgBuffs;
        VideoCapture picam;
        Mat buffImg;
        bool bInit;

};

ImgProducer::ImgProducer(ImgBuf* buf):picam(0)
{ 
    //capture the video from web cam

    if (!picam.isOpened() )  
    {
         cout << "Cannot open the web cam" << endl;
         bInit = false;
    }

    ImgBuffs = buf;
    bInit = true;
}

bool ImgProducer::Produce()
{
    // read a new frame from video
    if(false == picam.read(buffImg))
    {
         cout << "Cannot read a frame from video stream" << endl;
         return false;
    }

    ImgBuffs->PushImgBuff(buffImg);
    return true;
}

void* producer(void* arg)
{
    ImgProducer* producer = (ImgProducer* ) arg;

    while(!ProgramExit)
    {
        producer->Produce();
        printf("produce .. at %d in %s\n", __LINE__, __FUNCTION__);
    }
    pthread_exit(NULL);
}

void* consumer(void* arg)
{
    char img_name[50];
    static int i = 0;
    Mat imgOriginal;
    ImgBuf* buf = (ImgBuf* ) arg;
    while(!ProgramExit && i != 20)
    {
        buf->PopImgBuff(imgOriginal);
        sprintf(img_name, "img%d.jpg", i);
        if(false == imwrite(img_name, imgOriginal))
        {
            fprintf(stderr, "image write failed\n");
        }
        i++;
    }

    ProgramExit = true;

    pthread_exit(NULL);
}

int main( int argc, char** argv )
{
    pthread_mutex_t lock;

    if(pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr, "mutex init failed\n");
        return -1;
    }
#define NUM_THREADS 2

    pthread_t threads[NUM_THREADS];
    ImgBuf buf(&lock);
    ImgProducer imgProducer(&buf);

    pthread_create(&threads[0], NULL, producer, static_cast<void*>(&imgProducer)); 
    pthread_create(&threads[1], NULL, consumer, static_cast<void*>(&buf)); 
    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
    pthread_mutex_destroy(&lock);
    return 0;
}
