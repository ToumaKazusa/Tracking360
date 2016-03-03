#include <iostream>
#include <pthread.h>
#include "coordinates.h"
#include "PCA9685Driver.h"

#define DEBUG_360 true

using namespace cv;
using namespace std;

bool ProgramExit = false;
template<class T>
class ImgBuf
{
    enum {
        max_buff_size = 20,
    };
    public:
        bool PushImgBuff(T& buff);
        bool PopImgBuff(T& buff);
        void SetMutex(pthread_mutex_t* mutex);
        void SetCond(pthread_cond_t* cond);
        ImgBuf(pthread_mutex_t* mutex, pthread_cond_t* cond);
        ImgBuf();
    private:
        ImgBuf(const ImgBuf &obj);
        T _buff[max_buff_size];
        int head, tail, sz;
        bool debug;
        pthread_mutex_t* _mutex;
        pthread_cond_t*  _cond;;
};

template<class T>
ImgBuf<T>::ImgBuf()
{
    head = tail = sz = 0;
    _mutex = NULL;
    debug = DEBUG_360;
}

template<class T>
ImgBuf<T>::ImgBuf(pthread_mutex_t* mutex, pthread_cond_t* cond)
{
    head = tail = sz = 0;
    _mutex = mutex;
    _cond  = cond;
    debug = DEBUG_360;
}

template<class T>
void ImgBuf<T>::SetMutex(pthread_mutex_t* mutex)
{
    _mutex = mutex;
}

template<class T>
void ImgBuf<T>::SetCond(pthread_cond_t* cond)
{
    _cond = cond;
}

template<class T>
bool ImgBuf<T>::PushImgBuff(T& buf)
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
    pthread_cond_signal(_cond);

    return true;
}

template<class T>
bool ImgBuf<T>::PopImgBuff(T& buf)
{
    pthread_mutex_lock(_mutex);
    while(sz == 0)
    {
        if(debug)
            fprintf(stderr, "buffer underflow detected at %d in %s \n", 
                    __LINE__, __FUNCTION__);
        pthread_cond_wait(_cond, _mutex);
    }
    buf = _buff[tail++];
    tail %= max_buff_size;
    --sz;
    pthread_mutex_unlock(_mutex);

    return true;
}

class ImgProducer
{
    public:
        ImgProducer(ImgBuf<Mat>*);
        bool Produce(void);
    private:
        ImgProducer(const ImgBuf<Mat> &obj);
        ImgBuf<Mat>* ImgBuffs;
        VideoCapture picam;
        Mat buffImg;
        bool bInit;

};

ImgProducer::ImgProducer(ImgBuf<Mat>* buf):picam(0)
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
    Mat imgBGR2GRAY;
    cvtColor(buffImg, imgBGR2GRAY, COLOR_BGR2GRAY);
    ImgBuffs->PushImgBuff(imgBGR2GRAY);
    return true;
}

void* producer(void* arg)
{
    ImgProducer* producer = (ImgProducer* ) arg;

    while(!ProgramExit)
    {
        producer->Produce();
    }
    pthread_exit(NULL);
}

class ImageProcessor
{
    public:
        ImageProcessor(ImgBuf<Mat>*, ImgBuf<struct degree>*);
        void Process();
    private:
        Mat curImg, prevImg;
        ImgBuf<Mat>* _imgBuf; 
        ImgBuf<struct degree>* _cmdBuf;
        bool bPrevImgPresent;
};

ImageProcessor::ImageProcessor(ImgBuf<Mat>* imgBuf, ImgBuf<struct degree>* cmdBuf)
{ 
    _imgBuf = imgBuf;
    _cmdBuf = cmdBuf;
    bPrevImgPresent = false;
}

void ImageProcessor::Process()
{
    struct degree result;
    _imgBuf->PopImgBuff(curImg);
    if(bPrevImgPresent)
        Cameradegree (prevImg, curImg, result);
    prevImg = curImg.clone(); 
    bPrevImgPresent = true;
    printf("x --> %d, y --> %d at %d in %s", result.x, result.y, __LINE__, __FUNCTION__);

    _cmdBuf->PushImgBuff(result);
}


class ServoController
{
    public:
        void Sense();
        ServoController(ImgBuf<struct degree> *);
    private:
        ImgBuf<struct degree> *cmdBuf;
        struct degree current_pos;
        struct Track360MotorControlType actuator;
};

ServoController::ServoController(ImgBuf<struct degree>* buf)
{
    cmdBuf = buf;
    current_pos.x = 90;
    current_pos.y = 90;
    Track360MtrCtrl_Init(&actuator);
    Track360MtrCtrl_Pan(&actuator, current_pos.x);
    Track360MtrCtrl_Tilt(&actuator, current_pos.y);
}

void ServoController::Sense(void)
{
    struct degree cmd;
    cmdBuf->PopImgBuff(cmd);
    updatedegree(current_pos, cmd);
    printf("cmd --> x: %d y: %d\n", cmd.x, cmd.y);
    //if(current_pos.x != cmd.x)
    //{ 
    //    Track360MtrCtrl_Pan(&actuator, cmd.x);
    //    current_pos.x = cmd.x;
    //}
    if(current_pos.y != cmd.y)
    { 
        Track360MtrCtrl_Tilt(&actuator, cmd.y);
        current_pos.y = cmd.y;
    }
}

void* consumer(void* arg)
{
    static int i = 0;
    ImageProcessor* imgProc = (ImageProcessor* ) arg;
    while(!ProgramExit && i != 1000)
    {
        imgProc->Process();
        i++;
    }

    ProgramExit = true;

    pthread_exit(NULL);
}

void* actuate(void* arg)
{
    ServoController* controller = (ServoController*) arg;
    while(!ProgramExit)
    { 
        controller->Sense();
    }

    pthread_exit(NULL);
}

int main( int argc, char** argv )
{
    pthread_mutex_t lock, cmd_lock;
    pthread_cond_t buf_not_empty, cmd_ready_cond;

    if(pthread_mutex_init(&lock, NULL) != 0 ||
       pthread_mutex_init(&cmd_lock, NULL) != 0)
    {
        fprintf(stderr, "mutex init failed\n");
        return -1;
    }
    if(pthread_cond_init(&buf_not_empty, NULL)  != 0 ||
       pthread_cond_init(&cmd_ready_cond, NULL) != 0)
    {
        fprintf(stderr, "condition variable init failed\n");
        return -1;
    }

#define NUM_THREADS 3

    pthread_t threads[NUM_THREADS];

    ImgBuf<Mat> buf(&lock, &buf_not_empty);
    ImgBuf<struct degree> cmdBuf(&cmd_lock, &cmd_ready_cond);

    ImgProducer imgProducer(&buf);
    ImageProcessor imgProcessor(&buf, &cmdBuf);
    ServoController controller(&cmdBuf);

    pthread_create(&threads[0], NULL, producer, static_cast<void*>(&imgProducer)); 
    pthread_create(&threads[1], NULL, consumer, static_cast<void*>(&imgProcessor)); 
    pthread_create(&threads[2], NULL, actuate,  static_cast<void*>(&controller)); 
    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
    pthread_join(threads[2], NULL);
    pthread_mutex_destroy(&lock);
    pthread_mutex_destroy(&cmd_lock);
    pthread_cond_destroy(&buf_not_empty);
    pthread_cond_destroy(&cmd_ready_cond);
    return 0;
}
