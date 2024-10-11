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
