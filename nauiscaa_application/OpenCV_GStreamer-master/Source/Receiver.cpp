#include <iostream>
#include <gst/gst.h>

int main(int arg, char *argv[]) {
    GstElement *pipeline = nullptr;
    GstBus *bus = nullptr;
    GstMessage *msg = nullptr;

    // gstreamer initialization
    gst_init(&arg, &argv);

    // building pipeline
//    pipeline = gst_parse_launch(
//            "playbin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm",
//            nullptr);

 //   gst_parse_bin_from_description ("udpsrc port=5000 !application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink",
 //                                     true,
 //                                    error);

     pipeline = gst_parse_launch(
             "gst-launch-1.0 -v udpsrc port=5000 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink",
            nullptr);

    // start playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    //wait until error or EOS ( End Of Stream )
    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                     static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    // free memory
    if (msg != nullptr)
        gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}















///*
//* Receiver.cpp: Receiver for OpenCV_GStreamer example
//*
//* Copyright (C) 2019 Simon D. Levy
//*
//* MIT License
//*/

//#include "pch.h"

//#include <opencv2/opencv.hpp>
//using namespace cv;

//#include <iostream>
//using namespace std;

//int main()
//{
//    // The sink caps for the 'rtpjpegdepay' need to match the src caps of the 'rtpjpegpay' of the sender pipeline
//    // Added 'videoconvert' at the end to convert the images into proper format for appsink, without
//    // 'videoconvert' the receiver will not read the frames, even though 'videoconvert' is not present
//    // in the original working pipeline
////	VideoCapture cap("udpsrc port=5000 ! application/x-rtp,media=video,payload=26,clock-rate=90000,encoding-name=JPEG,framerate=30/1 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
////            CAP_GSTREAMER);
//    cout << "OpenCV version : " << CV_VERSION << endl;

////    VideoCapture cap("udpsrc port=5000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! decodebin ! videoconvert ! appsink",
////            CAP_ANY);
// //   VideoCapture cap("videotestsrc !  appsink0",CAP_GSTREAMER);
//    VideoCapture cap(0);

//    cerr << cap.get(CAP_PROP_FRAME_WIDTH)<<endl;

// //   bool res = cap.open("udpsrc port=5000 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! decodebin ! videoconvert ! autovideosink",
// //            CAP_GSTREAMER);
// //   cerr << res<<endl;
////    cerr << cap.getBackendName().c_str()<<endl;
//    if (!cap.isOpened()) {
//        cerr <<"VideoCapture not opened"<<endl;
//        exit(-1);
//    }
    
//    while (true) {

//        Mat frame;

//        cap.read(frame);

//        imshow("receiver", frame);

//        if (waitKey(1) == 27) {
//            break;
//        }
//    }

//    return 0;
//}
