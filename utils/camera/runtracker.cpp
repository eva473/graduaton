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

using namespace std;
using namespace cv;

Rect first_roi(Rect roi) {
	roi.x = 200;
	roi.y = 150;
	roi.width = 200;
	roi.height = 250;	
	return roi;
}

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

int main(int argc, char* argv[]){
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
    Rect roi;
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
            
	        //roi = selectROI("Image",frame,false,false);
		roi = first_roi(roi);
//////////////////////////////////////////////////////////////////////////

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
}
