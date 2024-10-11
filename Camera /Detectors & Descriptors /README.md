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

### Detectors vs Descriptors

•	A keypoint detector is an algorithm that chooses points from an image based on a local maximum of a function, such as the "cornerness" metric with the Harris detector. Keypoints are also known as referred to as interest points or salient points.
•	A descriptor is a vector of values, which describes the image patch around a keypoint. Techniques for generating descriptors range from comparing raw pixel values to more advanced methods like histograms of gradient orientations.
Descriptors help us to assign similar keypoints in different images to each other. As shown in the figure below, a set of keypoints in one frame is assigned keypoints in another frame such that the similarity of their respective descriptors is maximized and (ideally) the keypoints represent the same object in the image. In addition to maximizing similarity, a good descriptor should also be able to minimize the number of mismatches, i.e. avoid assigning keypoints that do not correspond to the same object.

![image](https://github.com/user-attachments/assets/9030b57f-156d-44a8-bda7-5a4dcee24d43)

### Types of Descriptors

**Descriptors based on Histograms of Oriented Gradients (HOG):** The basic idea behind HOG is to describe the structure of an object by the distribution of its intensity gradients in a local neighborhood. 
One of the best-known examples of the HOG family is the Scale-Invariant Feature Transform (SIFT)

**Binary Descriptors:** A much faster alternative to HOG-based methods is the family of binary descriptors, which provide a fast alternative at only slightly worse accuracy and performance. The central idea of binary descriptors is to rely solely on the intensity information (i.e. the image itself) and to encode the information around a keypoint in a string of binary numbers, which can be compared very efficiently in the matching step when corresponding keypoints are searched. 
Currently, the most popular binary descriptors are BRIEF, BRISK, ORB, FREAK, and KAZE (all available in the OpenCV library). 


#### Scale-Invariant Feature Transform (SIFT):
The Scale-Invariant Feature Transform (SIFT) was introduced in [1999 by David Lowe](https://home.cis.rit.edu/~cnspci/references/dip/feature_extraction/lowe1999.pdf).  SIFT combines a keypoint detector and a descriptor, and it operates through a five-step process, which is briefly summarized below.

1.	The initial step involves detecting keypoints in the image using a method known as "Laplacian-of-Gaussian (LoG)," which relies on second-order intensity derivatives. The LoG is applied across multiple scales of the image, primarily detecting blobs rather than corners. Along with a distinct scale level, keypoints are also assigned an orientation based on the intensity gradients in a local neighborhood around the keypoint.
Laplacian-of-Gaussian (LoG): 
The LoG operation works as follows: you start by slightly blurring the image using a Gaussian kernel. Next, you compute the sum of the second-order derivatives, known as the "Laplacian." This process helps to identify edges and corners within the image, which are useful for detecting keypoints. Since we're aiming to create a keypoint detector, additional steps are taken to suppress the edges. LoG is frequently utilized for detecting blobs.

2.	Next, the area around each keypoint is transformed by removing the orientation and thus ensuring a canonical orientation. Additionally, the area is resized to a 16 x 16 pixel patch, resulting in a normalized region.

![image](https://github.com/user-attachments/assets/04555c99-4614-4b1e-96ef-d9517ae2ec7a)



3.	Then, the orientation and magnitude of each pixel within the normalized patch are computed based on the intensity gradients Ix and Iy.
4.	In the fourth step, the normalized patch is divided into a grid of 4 x 4 cells. Within each cell, the orientations of pixels which exceed a threshold of magnitude are collected in  an 8-bin histogram.

![image](https://github.com/user-attachments/assets/dbcdc842-f8ee-429f-a3cc-82b66ec01b1a)

![image](https://github.com/user-attachments/assets/c4e12309-8bdd-426d-ac45-a57f7199d09c)

5.	Last, the 8-bin histograms of all 16 cells are concatenated into a 128-dimensional vector (the descriptor) which is used to uniquely represent the keypoint.


#### Advantages of SIFT: 
-	Its ability to robustly identify objects even among clutter and under partial occlusion. 
-	It is invariant to uniform changes in scale, to rotation, to changes in both brightness and contrast 
-	Partially invariant to affine distortions.
#### Drawbacks of SIFT:
-	Low speed.

  ### Binary Descriptors
A much faster alternative to HOG-based methods is the family of binary descriptors, which provide a fast alternative at only slightly worse accuracy and performance. The problem with HOG-based descriptors is that they are based on computing the intensity gradients, a computationally expensive process. This is why SIFT is generally not employed on devices with limited processing power, such as smartphones. Although there have been improvements, like the development of the similar SURF algorithm, which uses the less costly integral image instead of intensity gradients, real-time applications are still uncommon.

The core concept of binary descriptors is to rely solely on the intensity information (i.e. the image itself) and to encode the information around a keypoint in a string of binary numbers. This allows for highly efficient comparison during the matching process when searching for corresponding keypoints.  The most widely used binary descriptors today include BRIEF, BRISK, ORB, FREAK, and KAZE, all of which are available in the OpenCV library. A comparison of several region detectors can be found in the paper titled [Image matching using SIFT, SURF, BRIEF, and ORB: performance comparison for distorted images](https://arxiv.org/pdf/1710.02726), by (Karami E. et al., 2017).
From a high-level perspective, binary descriptors consist of three major parts:
1.	A sampling pattern which describes where sample points are located around the location of a keypoint.
2.	A method for orientation compensation, which removes the influence of rotation of the image patch around a keypoint location.
3.	A method for sample-pair selection, which generates pairs of sample points which are compared based on their intensity values. If the intensity of the first point is greater than that of the second, a '1' is added to the binary string; otherwise, a '0' is added. After comparing all point pairs in the sampling pattern, a complete binary string is formed (hence the family name of this descriptor class).

#### Binary Robust Invariant Scalable Keypoints (BRISK):
BRISK is a keypoint detection method that combines a FAST-based detector with a binary descriptor, which is generated through intensity comparisons from a specific sampling of each keypoint's neighborhood. Points of interest are identified across both the image and scale dimensions using a saliency criterion similar to the AGAST method.

The sampling pattern of BRISK is composed of a number of sample points (blue), where a concentric ring (red) around each sample point denotes an area where Gaussian smoothing is applied to avoid aliasing. As opposed to some other binary descriptors such as ORB or BRIEF, the BRISK sampling pattern is fixed. 

![image](https://github.com/user-attachments/assets/e648d380-2afb-46dd-b5a6-d129717101a9)

During sample pair selection, the BRISK algorithm differentiates between long- and short-distance pairs. The long-distance pairs (i.e. sample points with a minimal distance between each other on the sample pattern) are used for estimating the orientation of the image patch from intensity gradients, whereas the short-distance pairs are used for the intensity comparisons from which the descriptor string is assembled.

Mathematical description of pairs for BRISK algorithm:

![image](https://github.com/user-attachments/assets/88cdd477-9ef8-43fd-ad4c-59a978b27858)


First, we define set A of all possible pairings of sample points. Then, we extract the subset L from A for which the euclidean distance is above a pre-defined upper threshold. This set is the long-distance pairs used for orientation estimation. Lastly, we extract those pairs from A whose euclidean distance is below a lower threshold. This set S contains the short-distance pairs for assembling the binary descriptor string.
The following figure shows the two types of distance pairs on the sampling pattern for long pairs (left) and short pairs (right).

![image](https://github.com/user-attachments/assets/35e1c0b1-1a7d-4223-91d8-926e6568aa7f)


![image](https://github.com/user-attachments/assets/3a4c7fa2-4844-4229-9039-5d5fdd5a67a9)

First, the gradient strength between two sample points is computed based on the normalized unit vector that gives the direction between both points multiplied with the intensity difference of both points at their respective scales. Then, the keypoint direction vector g is computed from the sum of all gradient strengths.
Based on g, we can use the direction of the sample pattern to rearrange the short-distance pairings and thus ensure rotation invariance. Based on the rotation-invariant short-distance pairings, the final binary descriptor can then be constructed


![image](https://github.com/user-attachments/assets/5bb7c68c-148a-4980-adb5-eae9437a43fe)


After computing the orientation angle of the keypoint from g, we use it to make the short-distance pairings invariant to rotation. Then, the intensity between all pairs in SS is compared and used to assemble the binary descriptor we can use for matching.

#### Oriented FAST and Rotated BRIEF (ORB)

ORB is a combination of a keypoint detector and descriptor algorithms. It works in two steps:
1.	Keypoint detection using FAST - ORB starts by finding keypoints. Once the key points in the image had been located, ORB then calculates a corresponding feature vector for each keypoint in the image. The ORB algorithm creates feature vectors that only contain ones and zeros. For this reason, they're called the binary feature vectors.

![image](https://github.com/user-attachments/assets/190b7c07-268a-4cdf-84e5-197157b8b289)

Keypoints around the cat's eyes and at the edges of its facial features.
2.	Description using BRIEF - ORB uses BRIEF, which in turn identifies the objects in images using the feature vectors created which represents the patterns of intensity around a key point and vary according to a specific keypoint and its surrounding pixel area. Multiple feature vectors can be used to identify a larger area and even a specific object in an image.
ORB, is incredibly fast, and impervious to noise illumination, and image transformations such as rotations. 
A comparison of several region detectors can be found in the paper titled [Image matching using SIFT, SURF, BRIEF, and ORB: performance comparison for distorted images](https://arxiv.org/pdf/1710.02726), by (Karami E. et al., 2017).

### Descriptor Matching

Once you have located and described a set of keypoints for each image in a sequence of frames, the next step in the tracking process is to find the best fit for each keypoint in successive frames. In order to do so, you need to implement a suitable similarity measure so that your tracking algorithm can uniquely assign keypoint pairs. 
#### Sum of Absolute Differences (SAD)
The first distance function is the "Sum of Absolute Differences (SAD)". As shown in the equation below, the SAD takes two descriptor vectors, da and db, as input. The SAD is calculated by subtracting each component in db from the corresponding component in da. The absolute values of these differences are then summed up. The SAD norm is also known as the L1-norm.

#### Sum of Squared Differences (SSD)
The second distance function is the "Sum of Squared Differences (SSD)," which, like the SAD, calculates the differences between individual components of two descriptor vectors. The key distinction between SAD and SSD is that SSD sums the squared differences rather than the absolute differences. The SSD norm is also known as the L2-norm. 
Both norms are illustrated in the following figure.

![image](https://github.com/user-attachments/assets/56d626d1-4400-475a-b2f7-c461a987b54d)
