/*Object Tracking Using Kalman Filter by Praharsha Sirsi
*
*To the extent possible under law, the person who associated CC0 with
*Object Tracking Using Kalman Filter has waived all copyright and related or neighboring rights
*to the project.
*
*You should have received a copy of the CC0 legalcode along with this
*work.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.*/

/*Praharsha Sirsi (s8prsirs@stud.uni-saarland.de) (2557724)
Shruthi Yaddanapalli (shruthiyaddanapalli@gmail.com) (2551067)*/

//initial includes
#include <iostream>

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

// Vector
#include <vector>

using namespace std;

// >>>>> Color to be tracked
int MIN_H = 0;
int MAX_H = 179;
int MIN_S = 0;
int MAX_S = 255;
int MIN_V = 0;
int MAX_V = 255;
// <<<<< Color to be tracked

//Defnitions
int Window_Width = 512;
int Window_Height = 384;
int quadrant1_x = 0;
int quadrant1_y = 0;
int quadrant2_x = 512;
int quadrant2_y = 0;
int quadrant3_x = 0;
int quadrant3_y = 384;
int quadrant4_x = 512;
int quadrant4_y = 384;
//Definitions

//callback whenever the HSV is changed in control window
void trackbar_callback(int,void*){
}

void adjustHSV(){
	cv::namedWindow("Control", CV_WINDOW_NORMAL); //create a window called "Control"
	cv::resizeWindow("Control",Window_Width,Window_Height); //w=1024/4, h=768/4
	cv::moveWindow("Control",quadrant1_x,quadrant1_y); //2nd quadrant
	//Create trackbars in "Control" window
	cv::createTrackbar("LowH", "Control", &MIN_H, 179,trackbar_callback); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &MAX_H, 179,trackbar_callback);

	cv::createTrackbar("LowS", "Control", &MIN_S, 255,trackbar_callback); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &MAX_S, 255,trackbar_callback);

	cv::createTrackbar("LowV", "Control", &MIN_V, 255,trackbar_callback); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &MAX_V, 255,trackbar_callback);
}

/*
void mouse_click(int event_num, int x, int y, int flags, void *click_point){
cv::Point*Clicked_Point = (cv::Point*)click_point;
Clicked_Point->x = x;
Clicked_Point->y = y;
}*/


int main()
{
	// Camera frame
	cv::Mat frame;

	// >>>> Kalman Filter
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;

	unsigned int type = CV_64F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

	cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	//cv::Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<double>(0) = 1.0f;
	kf.measurementMatrix.at<double>(7) = 1.0f;
	kf.measurementMatrix.at<double>(16) = 1.0f;
	kf.measurementMatrix.at<double>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.processNoiseCov.at<double>(0) = 1e-2;
	kf.processNoiseCov.at<double>(7) = 1e-2;
	kf.processNoiseCov.at<double>(14) = 5.0f;
	kf.processNoiseCov.at<double>(21) = 5.0f;
	kf.processNoiseCov.at<double>(28) = 1e-2;
	kf.processNoiseCov.at<double>(35) = 1e-2;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	// <<<< Kalman Filter

	// Camera Index
	int idx = 0; //usually the webcam index is 0 on laptops

	// Camera Capture
	cv::VideoCapture cap;
	string user_input = " ";

	// >>>>> Camera Settings
	std::cout << "Enter 1 for WebCam \n" << "2 for Video File \n" << "and Press enter" << endl;	
	getline(cin,user_input);
	std::cout << "You entered: " << user_input << endl << endl;

	if(user_input.compare("1") == 0)
	{
		if (!cap.open(idx))
		{
			std::cout << "Webcam not connected.\n" << "Please verify\n";
			return EXIT_FAILURE;
		}
	}
	else if(user_input.compare("2") == 0)
	{
		std::cout << "Select Video File\n";
		string folder_name = "..\\VideoFiles\\";
		string filename1 = "NormalTrack.mp4";
		string filename2 = "ObstructionTrack.mp4";
		string filename3 = "Pedestrian.avi";
		string filename4 = "ObstructionTrackEdited.mp4";

		std::cout << "Enter 1 for " << filename1 << endl; 
		std::cout << "2 for " << filename2 << endl;
		std::cout << "3 for " << filename3 << endl;
		std::cout << "4 for " << filename4 << endl << endl;

		string select_file = " ";
		getline(cin,select_file);


		if(select_file.compare("1") == 0)
		{
			cap.open(folder_name + filename1);
			MIN_H = 0;
			MAX_H = 50;
			MIN_S = 170;
			MAX_S = 230;
			MIN_V = 165;
			MAX_V = 255;
		}
		else if(select_file.compare("2") == 0)
		{
			cap.open(folder_name + filename2);
			MIN_H = 0;
			MAX_H = 50;
			MIN_S = 185;
			MAX_S = 255;
			MIN_V = 150;
			MAX_V = 230;
		}
		else if(select_file.compare("3") == 0)
		{
			cap.open(folder_name + filename3);
			MIN_H = 90;
			MAX_H = 150;
			MIN_S = 0;
			MAX_S = 60;
			MIN_V = 60;
			MAX_V = 120;
		}
		else if(select_file.compare("4") == 0)
		{
			cap.open(folder_name + filename4);
			MIN_H = 0;
			MAX_H = 50;
			MIN_S = 185;
			MAX_S = 255;
			MIN_V = 150;
			MAX_V = 230;
		}
		else 
		{
			EXIT_FAILURE;
		}		
	}
	else
	{
		return EXIT_FAILURE;
	}

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);

	//cv::VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(1024,768),true);
	// <<<<< Camera Settings

	cout << "\nHit 'q' to exit...\n";

	char ch = 0;

	double ticks = 0;
	bool found = false;

	int notFoundCount = 0;

	//open control window
	adjustHSV();

	/*int framecount = 0;
	cv::Mat storedframes;
	bool initialstore = true;*/

	// >>>>> Main loop
	while ((ch != 'q' && ch != 'Q'))
	{
		double precTick = ticks;
		ticks = (double) cv::getTickCount();

		double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

		// Frame acquisition
		cap >> frame;		
		if(!frame.empty())
		{

			cv::Mat res;
			frame.copyTo( res );

			if (found)
			{
				// >>>> Matrix A
				kf.transitionMatrix.at<double>(2) = dT;
				kf.transitionMatrix.at<double>(9) = dT;
				// <<<< Matrix A

				//cout << "dT:" << endl << dT << endl;

				state = kf.predict();
				// cout << "State post:" << endl << state << endl;

				cv::Rect2d predRect;
				predRect.width = state.at<double>(4);
				predRect.height = state.at<double>(5);
				predRect.x = state.at<double>(0) - predRect.width / 2;
				predRect.y = state.at<double>(1) - predRect.height / 2;

				cv::Point2d center;
				center.x = state.at<double>(0);
				center.y = state.at<double>(1);
				cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

				cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
			}

			// >>>>> Noise smoothing
			cv::Mat blur;
			cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
			// <<<<< Noise smoothing

			// >>>>> HSV conversion
			cv::Mat frmHsv;
			cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
			// <<<<< HSV conversion

			// >>>>> Color Thresholding			
			cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
			cv::inRange(frmHsv, cv::Scalar(MIN_H, MIN_S, MIN_V), cv::Scalar(MAX_H, MAX_S, MAX_V), rangeRes);
			// <<<<< Color Thresholding

			// >>>>> Improving the result
			cv::erode(rangeRes, rangeRes, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::dilate(rangeRes, rangeRes, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));			

			cv::dilate(rangeRes, rangeRes, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::erode(rangeRes, rangeRes, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			// <<<<< Improving the result

			// Thresholding viewing
			cv::namedWindow("Threshold",CV_WINDOW_NORMAL);
			cv::imshow("Threshold", rangeRes);
			cv::resizeWindow("Threshold",Window_Width,Window_Height); //w=1024/4, h=768/4
			cv::moveWindow("Threshold",quadrant3_x,quadrant3_y); //3rd quadrant

			// >>>>> Contours detection
			vector<vector<cv::Point> > contours;
			cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			// <<<<< Contours detection

			// >>>>> Filtering
			vector<vector<cv::Point> > balls;
			vector<cv::Rect> ballsBox;
			for (size_t i = 0; i < contours.size(); i++)
			{
				cv::Rect bBox;
				bBox = cv::boundingRect(contours[i]);

				if(bBox.area()>=256)
				{
					balls.push_back(contours[i]);
					ballsBox.push_back(bBox);
				}

				/*float ratio = (float) bBox.width / (float) bBox.height;				
				if (ratio > 1.0f)
					ratio = 1.0f / ratio;

				// Searching for a bBox almost square
				if (ratio > 0.75 && bBox.area() >= 200)
				{
					balls.push_back(contours[i]);
					ballsBox.push_back(bBox);
				}*/
			}
			// <<<<< Filtering

			//cout << "Balls found:" << ballsBox.size() << endl;

			// >>>>> Detection result
			for (size_t i = 0; i < balls.size(); i++)
			{
				cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
				cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

				cv::Point center;
				center.x = ballsBox[i].x + ballsBox[i].width / 2;
				center.y = ballsBox[i].y + ballsBox[i].height / 2;
				cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

				stringstream sstr;
				sstr << "(" << center.x << "," << center.y << ")";
				cv::putText(res, sstr.str(),
					cv::Point(center.x + 3, center.y - 3),cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
			}
			// <<<<< Detection result

			// >>>>> Kalman Update
			if (balls.size() == 0)
			{
				notFoundCount++;
				//cout << "notFoundCount:" << notFoundCount << endl;
				if( notFoundCount >= 100 )
				{
					found = false;
				}
				/*else
				kf.statePost = state;*/
			}
			else
			{
				notFoundCount = 0;

				meas.at<double>(0) = ballsBox[0].x + ballsBox[0].width / 2;
				meas.at<double>(1) = ballsBox[0].y + ballsBox[0].height / 2;
				meas.at<double>(2) = (double)ballsBox[0].width;
				meas.at<double>(3) = (double)ballsBox[0].height;

				if (!found) // First detection!
				{
					
					// >>>> Initialization
					kf.errorCovPre.at<double>(0) = 1; // px
					kf.errorCovPre.at<double>(7) = 1; // px
					kf.errorCovPre.at<double>(14) = 1;
					kf.errorCovPre.at<double>(21) = 1;
					kf.errorCovPre.at<double>(28) = 1; // px
					kf.errorCovPre.at<double>(35) = 1; // px

					state.at<double>(0) = meas.at<double>(0);
					state.at<double>(1) = meas.at<double>(1);
					state.at<double>(2) = 0;
					state.at<double>(3) = 0;
					state.at<double>(4) = meas.at<double>(2);
					state.at<double>(5) = meas.at<double>(3);
					// <<<< Initialization

					found = true;
				}
				else
					kf.correct(meas); // Kalman Correction

				//cout << "Measure matrix:" << endl << meas << endl;
			}
			// <<<<< Kalman Update

			// Final result
			cv::namedWindow("Tracking",CV_WINDOW_NORMAL);
			cv::imshow("Tracking", res);
			cv::resizeWindow("Tracking",Window_Width,Window_Height); //w=1024/4, h=768/4
			cv::moveWindow("Tracking",quadrant2_x,quadrant2_y); //3rd quadrant


			/*framecount++;
			if(initialstore){
				storedframes = res;
				initialstore = false;
			}
			if(framecount>=10){
				addWeighted( storedframes, 0.5, res, 0.5, 0.0, storedframes);
				framecount = 0;
			}*/

			// User key
			ch = cv::waitKey(1);
		}
		else 
		{
			//printf("!!! cvQueryFrame failed: no frame\n");
			//cv::imwrite( "../Results/Results.jpg", storedframes );
			cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
			continue;
		}
	}
	// <<<<< Main loop

	return EXIT_SUCCESS;
}