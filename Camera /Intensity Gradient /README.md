## Intensity Gradient and Filtering - Locating Keypoints in an Image

The first step in the tracking process is locating keypoints in an image. Most types of keypoints rely on how brightness information is spread across the image. Locations where it changes quickly are considered to have a high intensity gradient, making them potential keypoints.

With a camera we cannot measure distance to an object directly, instead we can compute time-to-collision based on relative distance ratios on the image sensor. To do so, we need a set of locations on the image plane which can serve as stable anchors to compute relative distances between them. 

By examining the three patches in the figure below, which were extracted from an image of a highway driving scene, we can see that in the leftmost patch, there is a distinctive contrast between bright and dark pixels which resembles a line from the bottom-left to the upper-right. The patch in the middle resembles a corner formed by a group of very dark pixels in the upper-left. The rightmost patch looks like a bright blob that might be approximated by an ellipse.

![image](https://github.com/user-attachments/assets/4df4d248-7cb1-436b-8e3d-af7bf828c528)


To accurately locate a keypoint in an image, we must assign it a distinct coordinate in both the x and y directions. However, not all of the patches above are suitable for this purpose. While both the corner and the ellipse can be precisely positioned in both x and y, the line in the leftmost image cannot.

![image](https://github.com/user-attachments/assets/eef21a81-71de-48f3-8162-8e8944e87c95)


In the following, we will thus concentrate on detecting corners in an image. 

### The Intensity Gradient

In the examples above, the contrast between neighboring pixels provides the necessary information to precisely locate features like the corner in the middle patch. The specific color is irrelevant; what matters is the color difference between the pixels forming the corner, which should be as pronounced as possible. Ideally, a perfect corner would be made up of only black and white pixels.

The figure below shows the intensity profile of all pixels along the red line in the image and the intensity gradient, which is the derivative of image intensity.

![image](https://github.com/user-attachments/assets/afce86e0-a1d9-411e-bd12-acbd9a17bdce)


The intensity profile increases rapidly where there is a significant contrast between neighboring pixels. For instance, the lower part of the street lamp on the left side and the dark door exhibit a clear intensity difference compared to the lighter wall. To assign unique coordinates to the pixels where these changes occur, we could analyze the derivative of the intensity, represented by the blue gradient profile below the red line. Sudden changes in image intensity appear as distinct peaks and valleys in this gradient profile. By searching for these peaks both horizontally and vertically, we could identify points that exhibit a gradient peak in both directions and use them as keypoints with defined x and y coordinates. In the example patches, this approach works best for the corner, whereas an edge-like structure would have similar gradients at all positions without a distinct peak in either direction.

Based on these observations, the initial step in keypoint detection is to compute a gradient image. Mathematically, the gradient is the partial derivative of the image intensity in both the x and y directions. The figure below illustrates the intensity gradient for three example patches, with the gradient direction indicated by the arrows.

![image](https://github.com/user-attachments/assets/65014dcb-f6c7-4cd4-bbb7-651009e66198)

In equations (1) and (2), the intensity gradient is approximated by the intensity differences between neighboring pixels, divided by the distance between those pixels in x- and y-direction. Next, based on the intensity gradient vector, we can compute both the direction as well as the magnitude as given by the following equations:


![image](https://github.com/user-attachments/assets/5245438a-b4b5-4442-b35a-75b355eef732)

There are several methods to compute the intensity gradient, with the simplest being to calculate the intensity difference between neighboring pixels. However, this approach is highly sensitive to noise and is not recommended for practical use.


#### Illustration of Applying Derivatives for Edge Detection

We apply the first and second derivatives to the intensity curve obtained by comparing the intensity values, resulting in a curve like the one given below:


![image](https://github.com/user-attachments/assets/69320a18-d19a-4a6f-afa6-c8a228478f93)

For the first derivative, the locations of edges are determined by their local extrema. For the second derivative, the locations of the edges are determined by their zero crossings.


## Computing the Intensity Gradient

After gently smoothing the image to minimize noise, we can compute the intensity gradient in both the x and y directions. There are various methods used for gradient computation. One of the most well-known methods is the `Sobel` operator (introduced in 1968), though other options, like the `Scharr` operator, optimized for rotational symmetry, are also available.

The Sobel operator is based on applying small integer-valued filters both in horizontal and vertical direction. The operators are 3x3 kernels, one for the gradient in x and one for the gradient in y. Both kernels are shown below.

![image](https://github.com/user-attachments/assets/4ca1b52b-753f-4965-8e7d-885cce3fd4bc)

In the following code, one kernel of the Sobel operator is applied to an image. Note that it has been converted to grayscale to avoid computing the operator on each color channel. This code can be found in the gradient_sobel.cpp file.


```ruby

 // load image from file
    cv::Mat img;
    img = cv::imread("./img1.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // create filter kernel
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    // apply filter
    cv::Mat result_x;
    cv::filter2D(imgGray, result_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // show result
    string windowName = "Sobel operator (x-direction)";
    cv::namedWindow( windowName, 1 ); // create window 
    cv::imshow(windowName, result_x);
    cv::waitKey(0); // wait for keyboard input before continuing

```

The gradient image produced is displayed below. It illustrates that regions with strong local contrast, such as the cast shadow of the preceding vehicle, result in high values in the filtered image.

![image](https://github.com/user-attachments/assets/28fdf8d2-5dc2-4b00-8f7a-1f14816dd803)


Note that in the above code, only the Sx filter kernel has been applied for now, which is why the cast shadow only shows in x direction. Applying Sy to the image yields the following result:


![image](https://github.com/user-attachments/assets/1a5926f7-77ee-414f-86c8-8f6300b1082a)


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
