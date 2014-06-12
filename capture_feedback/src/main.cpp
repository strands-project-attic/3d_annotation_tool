#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
//TSA: I THINK THIS NEEDS TO BE CHANGED ^

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>
//TSA: I THINK THIS NEEDS TO BE CHANGED ^

#include <string.h>

#include <SDL/SDL.h>
#include <SDL/SDL_audio.h>
#include <queue>
#include <cmath>


const int FREQUENCY = 44100;

class Beeper
{
private:
	double v;
    double volume;
    double freq;
public:
    Beeper();
    ~Beeper();
    void beep(double freq, float volume);
    void generateSamples(Sint16 *stream, int length);
};

void audio_callback(void*, Uint8*, int);

Beeper::Beeper()
{
    SDL_AudioSpec desiredSpec;

    desiredSpec.freq = 44100;
    desiredSpec.format = AUDIO_S16SYS;
    desiredSpec.channels = 1;
    desiredSpec.samples = 2048;
    desiredSpec.callback = audio_callback;
    desiredSpec.userdata = this;

	volume = 0;
	freq = 200;

    SDL_AudioSpec obtainedSpec;

    // you might want to look for errors here
    SDL_OpenAudio(&desiredSpec, &obtainedSpec);

    // start play audio
    SDL_PauseAudio(0);
}

Beeper::~Beeper(){SDL_CloseAudio();}

void Beeper::generateSamples(Sint16 *stream, int length)
{
	for(int i = 0; i < length; i++){
		stream[i] = volume * std::sin(v * 2 * M_PI / FREQUENCY);
		v += freq;
	}
}

void Beeper::beep(double freq_, float volume_)
{
    SDL_LockAudio();
    freq = freq_;
	volume = 28000*volume_;
    SDL_UnlockAudio();
}

void audio_callback(void *_beeper, Uint8 *_stream, int _length)
{
    Sint16 *stream = (Sint16*) _stream;
    int length = _length / 2;
    Beeper* beeper = (Beeper*) _beeper;

    beeper->generateSamples(stream, length);
}

using namespace std;

int counter = 0;
float error_volume = 0;

void  cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("pointcloud %i\n",counter);
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg (*input, cloud);
    //TSA: I THINK THIS NEEDS TO BE CHANGED ^

	int width = cloud.width;
	int height = cloud.height;

	IplImage * img					= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	char * rgb_data					= (char *)(img->imageData);



	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = h*width + w;
			rgb_data[3*ind+0] = int(cloud.points[ind].b);
			rgb_data[3*ind+1] = int(cloud.points[ind].g);
			rgb_data[3*ind+2] = int(cloud.points[ind].r);
		}
	}

	cv::ORB orb = cv::ORB(400,1.2f, 3, 51, 0,2, cv::ORB::HARRIS_SCORE, 91);
	cv::Mat mimg(img);
	cv::Mat desc1;
	vector <cv::KeyPoint> kp1;
	orb(mimg, cv::Mat(), kp1, desc1);
/*
	for(int w = 0; w < width; w+=2){
		for(int h = 0; h < height; h+=2){
			int ind = h*width + w;
			if(cloud.points[ind].z == 0){
				rgb_data[3*ind+0] = 255;
				rgb_data[3*ind+1] = 0;
				rgb_data[3*ind+2] = 255;
			}
		}
	}
*/
	int good = 0;
	for(int i = 0; i < (int)kp1.size();i++){
		cv::KeyPoint curr = kp1.at(i);
		int w = curr.pt.x+0.5;
		int h = curr.pt.y+0.5;
		
		int ind = 640*h+w;

		if(cloud.points[ind].z > 0){
			cvCircle(img, cvPoint(w, h), 3, cvScalar(0, 255, 0, 0), 1, 8, 0);
			good++;
		}else{
			cvCircle(img, cvPoint(w, h), 3, cvScalar(0, 0, 255, 0), 1, 8, 0);
		}
	}

	printf("good: %i\n",good);
	if(good > 200){error_volume = 0;}
	else{
		error_volume = (200.0-float(good))/200.0;
	}
	cvShowImage("input rgb", img);
	cvWaitKey(30);
	cvReleaseImage( &img);
	counter++;
}


void * sound_thread( void *ptr )	{
	SDL_Init(SDL_INIT_AUDIO);
	Beeper b;
	while(true){
		float vol = error_volume;
		printf("vol = %f\n",vol);
		b.beep(200,vol);
		usleep(100);
	}
}

int main(int argc, char **argv)
{
	printf("starting testing software\n");
	ros::init(argc, argv, "capture_feedback");
	ros::NodeHandle n;
	ros::Subscriber sub2 = n.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
	pthread_t sound;
	pthread_create( &sound, NULL, sound_thread, 0);
	ros::spin();
	return 0;
}
