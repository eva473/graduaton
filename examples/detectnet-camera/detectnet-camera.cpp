
#include <stdio.h>      
#include <unistd.h>     
#include <fcntl.h>      
#include <errno.h>      
#include <termios.h>    
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cmath>
#include <ctype.h>
#include "kcftracker.hpp"

#include <dirent.h>
#include "gstCamera.h"
#include "glDisplay.h"

#include "detectNet.h"
#include "commandLine.h"

#include <signal.h>

using namespace std;
using namespace cv;

bool signal_recieved = false;
int angle(Point2f A, Point2f B) {
    float val = (B.y-A.y)/(B.x-A.x); // calculate slope between the two points
    if(B.x!=A.x)
    {
        val = atan(val); // find arc tan of the slope using taylor series approximation
        val = ((int)(val*180/CV_PI))% 360; // Convert the angle in radians to degrees
        if(B.x < A.x) val+=180;
        if(val < 0) val = 360 + val;
        return val;
    }
    else if(B.y > 0) return 90;    
    else return -90;
}
void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_recieved = true;
	}
}

int usage()
{
/*
	printf("usage: detectnet-camera [-h] [--network NETWORK] [--threshold THRESHOLD]\n");
	printf("                        [--camera CAMERA] [--width WIDTH] [--height HEIGHT]\n\n");
	printf("Locate objects in a live camera stream using an object detection DNN.\n\n");
	printf("optional arguments:\n");
	printf("  --help            show this help message and exit\n");
	printf("  --network NETWORK pre-trained model to load (see below for options)\n");
	printf("  --overlay OVERLAY detection overlay flags (e.g. --overlay=box,labels,conf)\n");
	printf("                    valid combinations are:  'box', 'labels', 'conf', 'none'\n");
     printf("  --alpha ALPHA     overlay alpha blending value, range 0-255 (default: 120)\n");
	printf("  --camera CAMERA   index of the MIPI CSI camera to use (e.g. CSI camera 0),\n");
	printf("                    or for VL42 cameras the /dev/video device to use.\n");
     printf("                    by default, MIPI CSI camera 0 will be used.\n");
	printf("  --width WIDTH     desired width of camera stream (default is 1280 pixels)\n");
	printf("  --height HEIGHT   desired height of camera stream (default is 720 pixels)\n");
	printf("  --threshold VALUE minimum threshold for detection (default is 0.5)\n\n");

	printf("%s\n", detectNet::Usage());
*/
	return 0;
}

int main( int argc, char** argv )
{
    Rect roi;
int area[50]={-1};
   int a=0;
   int max=-1;
   int max_indx=-1;
	/*
	 * parse command line
	 */
	commandLine cmdLine(argc, argv);

	if( cmdLine.GetFlag("help") )
		return usage();


	/*
	 * attach signal handler
	 */
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("\ncan't catch SIGINT\n");


	/*
	 * create the camera device
	 */
	gstCamera* camera = gstCamera::Create(cmdLine.GetInt("width", 640),
								   cmdLine.GetInt("height", 480),
								   cmdLine.GetString("camera"));

	if( !camera )
	{
		printf("\ndetectnet-camera:  failed to initialize camera device\n");
		return 0;
	}
	
	printf("\ndetectnet-camera:  successfully initialized camera device\n");
	printf("    width:  %u\n", camera->GetWidth());
	printf("   height:  %u\n", camera->GetHeight());
	printf("    depth:  %u (bpp)\n\n", camera->GetPixelDepth());
	

	/*
	 * create detection network
	 */
	detectNet* net = detectNet::Create(argc, argv);
	
	if( !net )
	{
		printf("detectnet-camera:   failed to load detectNet model\n");
		return 0;
	}

	// parse overlay flags
	const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));
	

	/*
	 * create openGL window
	 */
	glDisplay* display = glDisplay::Create();

	if( !display ) 
		printf("detectnet-camera:  failed to create openGL display\n");


	/*
	 * start streaming
	 */
	if( !camera->Open() )
	{
		printf("detectnet-camera:  failed to open camera for streaming\n");
		return 0;
	}
	
	printf("detectnet-camera:  camera open for streaming\n");
	
	
	/*
	 * processing loop
	 */
	float confidence = 0.0f;
	int mmmode=0;
	while( !signal_recieved && mmmode==0 )
	{
		// capture RGBA image
		float* imgRGBA = NULL;
		
		if( !camera->CaptureRGBA(&imgRGBA, 1000) )
			printf("detectnet-camera:  failed to capture RGBA image from camera\n");

		// detect objects in the frame
		detectNet::Detection* detections = NULL;
	
		const int numDetections = net->Detect(imgRGBA, camera->GetWidth(), camera->GetHeight(), &detections, overlayFlags);
		
		if( numDetections > 0 )

		{

			printf("%i objects detected\n", numDetections);

		

			for( int n=0; n < numDetections; n++ )

			{

				//printf("detected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);

				//printf("bounding box %i  (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());

 

				if(detections[n].ClassID==1){

					if((detections[n].Left+detections[n].Width()/2>200&&detections[n].Left+detections[n].Width()/2<500)&&(detections[n].Top+detections[n].Height()/2>100&&detections[n].Top+detections[n].Height()/2<400)){

					area[a++]=detections[n].Width()*detections[n].Height();

					}

				}

			}

		}	

		if(area[0]!=-1){

   			for(int i=0;i<a;i++){

				if(area[i]>max){

					max=area[i];

					max_indx=i;

        			}

    			}
			roi.x=detections[max_indx].Left+detections[max_indx].Width()/2-100;
			roi.y=detections[max_indx].Top+detections[max_indx].Height()/3-125;//upper
			//roi.y=detections[max_indx].Top+detections[max_indx].Height()/2-125;
			roi.width = 200;
			roi.height = 250;
			//roi.width = detections[max_indx].Width();
			//roi.height = detections[max_indx].Height();
			mmmode=1;

		}

		else{

			mmmode=0;

		}
		// update display
		if( display != NULL )
		{
			// render the image
			display->RenderOnce(imgRGBA, camera->GetWidth(), camera->GetHeight());

			// update the status bar
			char str[256];
sprintf(str, "Image");
			//sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
			display->SetTitle(str);

			// check if the user quit
			if( display->IsClosed() )
				signal_recieved = true;
		}

		// print out timing info
		net->PrintProfilerTimes();
	}
printf("detectnet-camera:  shutting down...\n");
	
	SAFE_DELETE(camera);
	SAFE_DELETE(display);
	SAFE_DELETE(net);

	printf("detectnet-camera:  shutdown complete.\n");
int  serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY );
if(serial_port==-1){
	cout<<"unconnected"<<endl;
}
else{
	cout<<"succ"<<serial_port<<endl;
}
	fcntl(serial_port, F_SETFL, 0);

	/*Define the POSIX structure*/
	struct termios serial_options;

	/*Read the attribute structure*/
	tcgetattr(serial_port, &serial_options);

	/*Set the baud rate of the port  to 9600*/
	cfsetispeed(&serial_options, B115200);
	cfsetospeed(&serial_options, B115200);
        	serial_options.c_cflag |= (CLOCAL | CREAD);

	/*Define other parameters in order to  realize the 8N1 standard*/
	serial_options.c_cflag &= ~PARENB;
	serial_options.c_cflag &= ~CSTOPB;
	serial_options.c_cflag &= ~CSIZE;
	serial_options.c_cflag |= CS8;
	
	/*Apply the new attributes */
	tcsetattr(serial_port, TCSANOW, &serial_options);

    if (argc > 5) return -1;

    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = false;
    bool LAB = false;
    bool track=false;
    for(int i = 0; i < argc; i++){
        if ( strcmp (argv[i], "hog") == 0 )
            HOG = true;
        if ( strcmp (argv[i], "fixed_window") == 0 )
            FIXEDWINDOW = true;
        if ( strcmp (argv[i], "singlescale") == 0 )
            MULTISCALE = false;
        if ( strcmp (argv[i], "show") == 0 )
            SILENT = false;
        if ( strcmp (argv[i], "lab") == 0 ){
            LAB = true;
            HOG = true;
        }
        if ( strcmp (argv[i], "gray") == 0 )
            HOG = false;
    }

    // Create KCFTracker object
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

    // Frame readed
    Mat frame;

    // Tracker results
    Rect result;

    //Read video
    VideoCapture cap(0);

    // Frame counter
    int nFrames = 0;
    double apceValue = 0;
    double resMax =0;
    vector<double> preApce;
    vector<double> preResMax;
/////////////////////////////////////////////////////////////
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 40;
    bool needToInit = true;
    bool nightMode = false;
    namedWindow( "LK Demo", 0 );
    namedWindow( "Histrogram", 0 );
    Mat gray, prevGray;
    Mat hist(400, 500, CV_8UC3, Scalar(0,0,0));
    vector<Point2f> points[2];

int max_index=0;
char buf[4];
int mode=0;
int mode_y=0;

/////////////////////////////////////////////////////////
    while (cap.read(frame)){

        // First frame, give the groundtruth to the tracker
        if (nFrames == 0) {
            
	       // roi = selectROI("Image",frame,false,false);
		//roi = first_roi(roi);
//////////////////////////////////////////////////////////////////////////
//printf("****%d******\n",frame.rows);
//printf("****%d******\n",frame.cols);
        if( needToInit )
        {
            
			//points[1].x=frame_roi.cols/2;//////vector not like this
			//points[1].y=frame_roi.rows/2;
points[1].push_back(Point2f(roi.x + roi.width/2 , roi.y + roi.height/4));
circle( frame, points[1][0], 3, Scalar(0,255,0), -1, 8);

        }
//////////////////////////////////////////////////////////////////////////
            tracker.init(roi,frame);
            rectangle(frame , Point(roi.x , roi.y) , Point(roi.x + roi.width , roi.y + roi.height),Scalar(0,0,255),2);
            write(serial_port, "t", 1);
		cout << "t" << endl;
cout<<"x: "<<roi.x<<"~"<<roi.x+roi.width<<endl;
cout<<"y: "<<roi.y<<"~"<<roi.y+roi.height<<endl;
 	    }// Update
        else{
if((max_index==0)||(max_index>4&&max_index<14)||(max_index>22&&max_index<32)){
int r_result_x=result.x;
int r_result_y=result.y;
            result = tracker.update(frame);
            rectangle( frame, Point(result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 2, 8 );
//cout<<result.y+result.height/2<<"*"<<endl;
if(result.y+result.height/2>300&&mode_y!=1){
	write(serial_port, "x", 1);

tcdrain(serial_port);
cout<<" x   ";
mode_y=1;
}
else if(result.y+result.height/2<100&&mode_y!=2){
	write(serial_port, "z", 1);
tcdrain(serial_port);
cout<<" z   ";
mode_y=2;
}
else if((result.y+result.height/2>=200)&&(result.y+result.height/2<=300)&&(mode_y!=3)){
	write(serial_port, "y", 1);
tcdrain(serial_port);
cout<<" y   ";
mode_y=3;
}

cout<<frame.cols/2-(result.x+result.width/2) << endl;
if((frame.cols/2-(result.x+result.width/2)>50)&&mode!=2){
	write(serial_port, "b", 1);
tcdrain(serial_port);
cout<<" b"<<endl;
mode=2;
}else if(((result.x+result.width/2)-frame.cols/2>50)&&mode!=3){
	write(serial_port, "d", 1);
tcdrain(serial_port);
cout<<" d"<<endl;
mode=3;
}
else if(frame.cols/2-(result.x+result.width/2)<=50&&(result.x+result.width/2)-frame.cols/2<=50&&(mode!=5)){
	write(serial_port,"c",1);
tcdrain(serial_port);
cout<<" c"<<endl;
mode=5;
}
/*
if(r_result_x-result.x>60){
	write(serial_port, "a", 1);

tcdrain(serial_port);
cout<<"a"<<endl;
}
if(r_result_x-result.x<60&&r_result_x-result.x>20){
	write(serial_port, "b", 1);
tcdrain(serial_port);
cout<<"b"<<endl;
}
if(result.x-r_result_x>60&&result.x-r_result_x>20){
	write(serial_port, "d", 1);
tcdrain(serial_port);
cout<<"d"<<endl;
}
if(result.x-r_result_x>60){
	write(serial_port, "e", 1);
tcdrain(serial_port);
cout<<"e"<<endl;
}
else{
	write(serial_port,"c",1);
tcdrain(serial_port);
cout<<"c"<<endl;
}
*/
}
/////////////////////////////////////////////////////

cvtColor(frame, gray, COLOR_BGR2GRAY);

points[0].push_back(Point2f(result.x+result.width/2,result.y+result.height/4));
circle( frame, points[0][0], 3, Scalar(0,255,0), -1, 8);
//cout<<"**"<<points[0][0]<<endl;

        if( !points[0].empty() )
        {

            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k;

               
if((abs(points[0][0].x-points[1][0].x)>10)||(abs(points[0][0].y-points[1][0].y)>10)){
                line(frame, points[0][0], Point(points[0][0].x + ((points[1][0].x - points[0][0].x)*10),points[0][0].y + ((points[1][0].y - points[0][0].y)*10)), Scalar(255,0,0),3);
                circle(frame, points[1][0], 3, Scalar(0,255,0), -1, 8);
}
        }

        int gradient_hist[36];//={0};
        for(int j=0;j<36;j++)
            gradient_hist[j]=0;
        needToInit = false;
        if(!points[0].empty())
        {
            
                int theata,r;

if((abs(points[0][0].x-points[1][0].x)>10)||(abs(points[0][0].y-points[1][0].y)>10)){
                theata = angle(points[0][0],points[1][0]);
                r = norm(points[0][0] - points[1][0]);
                int index = int(theata/10);
//cout<<"**"<<index<<endl;
                gradient_hist[index] += 1;
}
            
        }
        //hist = Scalar::all(0); 
   /*
        for(int l=0;l<35;l++)
        {
            int x1,y1,x2,y2;
            x1 = l*10;
            x2 = x1+10;
            y1 = gradient_hist[l];
            y2 = gradient_hist[l+1];
            line(image, Point(x1,y1), Point(x2,y2), Scalar(255,0,0),1);
        }
*/
int max=0;
max_index=0;

for(int l=0;l<36;l++)
{
	if(max<gradient_hist[l]){
		max=gradient_hist[l];
		max_index=l;
	}
}
//cout<<max_index<<endl;
/*
if(max_index==0)
        cout<<"o"<<endl;
if((max_index>0&&max_index<3)||(max_index>33&&max_index<=36))
        cout<<"->"<<endl;
if(max_index>15&&max_index<21)
        cout<<"<-"<<endl;
if(max_index>=3&&max_index<=15)
        cout<<"v"<<endl;
if(max_index>=21&&max_index<=33)
        cout<<"^"<<endl;
*/
        //imshow("LK Demo", result_roi);
        //imshow("Histrogram",hist);
        //waitKey(30);
        
        ////std::swap(points[1], points[0]);
        ////cv::swap(prevGray, gray);

points[1].clear();
points[1].push_back(Point2f(points[0][0].x,points[0][0].y));
points[0].clear();
prevGray=gray.clone();
////////////////////////////////////////////

            //preApce.push_back(apceValue);
            //preResMax.push_back(resMax);
            double addApce = 0;
            double addResMax = 0;
            double comApce = 0;
            double comResMax = 0;
            //cout<<"apceValue = "<<apceValue<<endl;
            //cout<<"resMax = "<<resMax<<endl;
            preApce.push_back(apceValue);
            preResMax.push_back(resMax);
            //cout<<"preResMax.size() = "<<preResMax.size()<<endl;

        /*    for (int j = 0; j < sz; j++) {
                *//* cout<<preApce[j]<<endl;
                       cout<<preResMax[j]<<endl;*//*
                    addApce += preApce[j];
                    addResMax += preResMax[j];
            }*/

            //addApce = accumulate(preApce.begin(), preApce.end(),0);
            //addResMax = accumulate(preResMax.begin(), preResMax.end(),0);
            int sz =  preApce.size()-1;
            for(int i=0;i<sz;i++){

                addApce +=preApce[i];
                addResMax +=preResMax[i];
            }
            //cout<<"addResMax = "<<addResMax<<endl;


            if(sz>0) {

                addApce = addApce / sz;
                addResMax = addResMax / sz;

                //cout<<"sz ="<<sz<<endl;
                comApce = 0.5 * addApce;
                comResMax = 0.5 * addResMax;
               // cout<<"comApce = "<<comApce<<endl;
                //cout<<"comResMax = "<<comResMax<<endl;

                if (apceValue > comApce && resMax > comResMax) {

                    //std::cout << "tracking was successed!" << std::endl;
                    track = true;

                } else {

                    //std::cout << "tracking was failed!" << std::endl;
                    track = false;
                }

            }

	    }

        nFrames++;


        if (!SILENT){
            imshow("Image", frame);
            waitKey(100);
        }
    }
  
close(serial_port);

	/*
	 * destroy resources
	 */
	
	return 0;
}
