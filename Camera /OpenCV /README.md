# OpenCV Library Overview

![image](https://github.com/user-attachments/assets/d7c28ef2-0287-4acb-afaa-52365d0e170c)


Throughout the camera course, you will be using the [OpenCV]( https://opencv.org/), which is a cross-platform computer vision library which was originally developed in the year 2000 to provide a common infrastructure for computer vision applications and to accelerate the use of machine vision in science and engineering projects.
The library has more than 2500 algorithms that can be used to detect and recognize faces, identify objects, classify human actions in videos, track camera movements, track moving objects, perform machine learning and many more. OpenCV is written natively in C++ but has interfaces to Python, Java and Matlab as well. In the camera course, you will be using the C++ version of OpenCV.
The major advantage in using the OpenCV library is that you will be able to leverage a well-tested set of state-of-the-art computer vision algorithms. Without having to concentrate on the actual implementation of computer vision concepts such as Sobel operators, keypoint detection or machine learning you can use them right out of the box and concentrate on combining them in the right way to develop a working software prototype. 

To use the library in your code, the following header has to be included:
```
#include "opencv2/core/core.hpp"
```
The highgui module contains user interface functions that can be used to display images or take simple user input. To use the library in your code, the following header has to be included:
```
#include "opencv2/highgui/highgui.hpp"
```
In this project, basic functions such as ```cv::imshow``` will be used to display images in a window.
The imgproc (image processing) module contains basic transformations on images, such as image filtering, geometric transformations, feature detection and tracking. To use the library in your code, the following header has to be included:
```
#include "opencv2/imgproc/imgproc.hpp"
```
The features2d module contains algorithms for detecting, describing, and matching keypoints between images. To use the library in your code, the following header has to be included:
```
#include "opencv2/features2d/features2d.hpp"
```

## The OpenCV Matrix Datatype
The basic data type in OpenCV to store and manipulate images is the ```cv::Mat datatype```. It can be used for arrays of any number of dimensions. The data stored in ```cv::Mat``` is arranged in a so-called raster scan order. For a two-dimensional array (such as a grayscale image), this means that the data is organized into rows, and each row appears one after the other. A three-dimensional array (e.g. a color image) is arranged in planes, where each plane is filled out row by row, and then the planes are packed one after the other.
The data inside a ```cv::Mat``` variable can be either single numbers or multiple numbers. In the case of multiple numbers (e.g. represented by ```cv::Scalar```), the matrix is referred to as a multichannel array. There are several ways to create and initialize a ```cv::Mat``` variable. 

