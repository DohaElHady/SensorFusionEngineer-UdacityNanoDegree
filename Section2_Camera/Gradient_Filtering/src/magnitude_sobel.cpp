#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    // ToDo : Add your code here
    cv::Mat imgBlurred;
    cv::GaussianBlur(imgGray, imgBlurred, cv::Size(5, 5), 0);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
    float sobel_y[9] = {+1, +2, +1,
                         0,  0,  0,
                        -1, -2, -1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);


    // apply filter using the OpenCv function filter2D()
    // ToDo : Add your code here
    cv::Mat result_x, result_y;
    cv::filter2D(imgBlurred, result_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(imgBlurred, result_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);    
    

    // compute magnitude image based on the equation presented in the lesson 
    // gradient magnitude = sqrt( Gx^2 + Gy^2 )
    // ToDo : Add your code here
    cv::Mat magnitude;
    magnitude = cv::Mat::zeros(result_x.size(), result_x.type());
    for (int i = 0; i < result_x.rows; i++)
    {
        for (int j = 0; j < result_x.cols; j++)
        {
            float gx = result_x.at<uchar>(i, j);
            
            float gy = result_y.at<uchar>(i, j);
            magnitude.at<uchar>(i, j) = static_cast<uchar>(sqrt(gx * gx + gy * gy));
        }
    }
    

    // show result
    string windowName = "Gradient Magnitude";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}