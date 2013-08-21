#ifndef RGBD_GRABBER_HH
#define RGBD_GRABBER_HH

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "ros/ros.h"

#include <string>

#include <iostream>
#include <fstream>
#include <deque>

class RGBDGrabber
{


public:
    RGBDGrabber();
    ~RGBDGrabber();

    void pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr& msg);
    void colorImageCallback (const sensor_msgs::Image::ConstPtr& img);
    void depthImageCallback (const sensor_msgs::Image::ConstPtr& img);

    void setSaveOneFrame(const bool&);
    void setSaveFrameSeq(const bool&);
    void setFrameSkip(const int&);
    int  getFrameSkip();
    void increaseFrameSkip();
    void decreaseFrameSkip();
    std::string getFolderName();

    bool createNewFolder();


    static std::string getDateTime();

private:
    volatile bool                               m_bSaveOneFrame, m_bSaveFrameSequence, m_bCreateCVWindow, m_bLensCovered;
    int                                         m_iSequenceNumber;
    int                                         m_iFrameSkip;
    std::string                                 m_sCurrentFolder;
    std::ofstream                               m_fIndexFile;
    double                                      m_dLastTimestamp;
    std::deque<std::pair<int, double> >         m_DepthSequenceAndTimestamp;
    std::deque<std::pair<int, double> >         m_RGBSequenceAndTimestamp;
    int                                         m_iImageSyncQueueLength;
    double                                      m_dImageSyncTimout; // im seconds
    double                                      m_dImageSyncBetweenFrames; // im seconds
    double                                      m_dAvgPixelValueWhenLensCovered;
};

#endif // DATA_COMPRESION_NODE_HH
