
## Harris Corner Detection
The Harris corner detector is based on detecting locations in an image that show strong intensity gradients

The following figure shows an image patch which consists of line structures on a homogeneously colored background. A red arrow indicates that no unique position can be found in this direction. The green arrow expresses the opposite. As can be seen, the corner is the only local structure that can be assigned a unique coordinate in x and y.

![image](https://github.com/user-attachments/assets/e1f96a00-6a4a-41c8-8ed9-2cbc36730f9a)

In order to locate a corner, we consider how the content of the window would change when shifting it by a small amount. For case (a) in the figure above, there is no measurable change in any coordinate direction at the current location of the red window W whereas for (b), there will be significant change into the direction orthogonal to the edge and no change when moving into the direction of the edge. In case of (c), the window content will change in any coordinate direction.

The idea of locating corners by means of an algorithm is to find a way to detect areas with a significant change in the image structure based on a displacement of a local window W. Generally, a suitable measure for describing change mathematically is the sum of squared differences (SSD), which looks at the deviations of all pixels in a local neighborhood before and after performing a coordinate shift. The equation below illustrates the concept.

![image](https://github.com/user-attachments/assets/fe1892ba-0c69-450a-9fb9-bef932926a9b)

After shifting the Window W by an amount u in x-direction and v in y-direction the equation sums up the squared differences of all pixels within W at the old and at the new window position. In the following we will use some mathematical transformations to derive a measure for the change of the local environment around a pixel from the general definition of the SSD.

In the first step, based on the definition of E(u, v) above, we will at first make a Taylor series expansion of I(x + u, y + v). For small values of u and v, the first-order approximation is sufficient, which leads to the following expression.

![image](https://github.com/user-attachments/assets/db122d4a-d2a7-42ec-89f5-b0f077ab675d)

In the second step, we will now insert the approximated expression of I(x+u,y+v) into the SSD equation above, which simplifies to the following form:

![image](https://github.com/user-attachments/assets/400b062e-6cf5-450a-9b76-b63406675a3d)

The result of our mathematical transformations is a matrix H, which can now be conveniently analyzed to locate structural change in a local window W around every pixel position u,v in an image. In the literature, the matrix H is often referred to as covariance matrix.

To do this, it helps to visualize the matrix H as an ellipse, whose axis length and directions are given by its eigenvalues and eigenvectors. As can be seen in the following figure, the larger eigenvector points into the direction of maximal intensity change, whereas the smaller eigenvector points into the direction of minimal change. So in order to identify corners, we need to find positions in the image which have two significantly large eigenvalues of H.

![image](https://github.com/user-attachments/assets/5b37ad19-4947-445c-baad-ab3798e688a6)

how they can be computed from H:

![image](https://github.com/user-attachments/assets/588c66ba-17d3-4c1a-bb76-987fbfce1e55)

In addition to the smoothing of the image before gradient computation, the Harris detector uses a Gaussian window w(x,y) to compute a weighted sum of the intensity gradients around a local neighborhood. The size of this neighborhood is called scale in the context of feature detection and it is controlled by the standard deviation of the Gaussian distribution.

![image](https://github.com/user-attachments/assets/7224a0a4-be29-47b9-a4a3-e33c77d87396)

As can be seen, the larger the scale of the Gaussian window, the larger the feature below that contributes to the sum of gradients. By adjusting scale, we can thus control the keypoints we are able to detect.


The Harris corner detector evaluates the following expression to derive a corner response measure at every pixel location with the factor k being an empirical constant which is usually in the range between k = 0.04 - 0.06.

![image](https://github.com/user-attachments/assets/ec470c8d-5f40-4d51-99ff-e66ddb885e8f)

The following code computes the corner response for a given image and displays the result:

``` ruby
// load image from file
    cv::Mat img;
    img = cv::imread("./img1.png");

    // convert image to grayscale
    cv::Mat imgGray; 
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Detector parameters
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04; // Harris parameter (see equation for details)
    
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(imgGray.size(), CV_32FC1 );
    cv::cornerHarris( imgGray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow( windowName, 4 );
    cv::imshow( windowName, dst_norm_scaled );
    cv::waitKey(0);
```

The result can be seen below : The brighter a pixel, the higher the Harris corner response.

![image](https://github.com/user-attachments/assets/1d514b64-bd80-4cee-8b17-d49b2f01c3f1)

In order to locate corners, we now have to perform a **non-maxima suppression (NMS)** to:

- Ensure that we get the pixel with maximum cornerness in a local neighborhood and
- Prevent corners from being too close to each other as we prefer an even spread of corners throughout the image.

In nms_example.cpp the code example illustrates the basic principle behind non-maximum suppression. The idea is to reduce the intensities (e.g. corner response) in a local neighborhood in such a way that only the strongest response remains. 

### Disadvantages of the Harris Detector
One of the disadvantages of the Harris detector is that it does not work well with certain transformations of the image content. These might be rotations or scale (i.e. size) changes or even perspective transformations.

## Overview of Popular Keypoint Detectors
Keypoint detectors are a very popular research area and thus a large number of powerful algorithms have been developed over the years. Applications of keypoint detection include such things as object recognition and tracking, image matching and panoramic stitching as well as robotic mapping and 3D modeling. Detectors can be compared for their detection performance and their processing speed.

The Harris detector along with several other "classics" belongs to a group of traditional detectors, which aim at maximizing detection accuracy. In this group, computational complexity is not a primary concern. The following list shows a number of popular classic detectors:

- 1988 Harris Corner Detector (Harris, Stephens)
- 1996 Good Features to Track (Shi, Tomasi)
- 1999 Scale Invariant Feature Transform (Lowe)
- 2006 Speeded Up Robust Features (Bay, Tuytelaars, Van Gool)

In recent years, a number of faster detectors have been developed which aim at real-time applications on smartphones and other portable devices. The following list shows the most popular detectors belonging to this group:

- 2006 Features from Accelerated Segment Test (FAST) (Rosten, Drummond)
- 2010 Binary Robust Independent Elementary Features (BRIEF) (Calonder, et al.)
- 2011 Oriented FAST and Rotated BRIEF (ORB) (Rublee et al.)
- 2011 Binary Robust Invariant Scalable Keypoints (BRISK) (Leutenegger, Chli, Siegwart)
- 2012 Fast Retina Keypoint (FREAK) (Alahi, Ortiz, Vandergheynst)
- 2012 KAZE (Alcantarilla, Bartoli, Davidson)
