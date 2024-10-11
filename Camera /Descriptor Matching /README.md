# Descriptor Matching
Once we locate and describe a set of keypoints for each image in a sequence of frames, the next step in the tracking process is to find the best fit for each keypoint in successive frames. In order to do so, we need to implement a suitable ***similarity measure*** so that the tracking algorithm can uniquely assign keypoint pairs.

### Sum of Absolute Differences (SAD)
The first distance function is the "Sum of Absolute Differences (SAD)". As shown in the equation below, the SAD takes as input two descriptor vectors ***da*** and ***db***. . The SAD is computed by subtracting from every component in ***da***  the corresponding component at the same position in ***db***. Then, the absolute value of the respective result is summed up. The SAD norm is also referred to as L1-norm in the literature.

### Sum of Squared Differences (SSD)
The second distance function is the "Sum of Squared Differences (SSD)", which is similar to the SAD in the sense that differences between individual components of two descriptor vectors are computed. However, the key difference between SAD and SSD is that the latter sums the squared differences instead of the absolute differences. In the literature, the SSD norm is also referred to as L2-norm.

![image](https://github.com/user-attachments/assets/eea0191c-4848-4f94-8d8a-8ef51fe55a44)

There are several ways of explaining the differences between SAD and SSD. One helpful approach is to look at both norms from a geometrical perspective. In the following figure, a two-dimensional feature space is considered. In it, there are two feature vectors d1 and d2, each of which consists of an (a,b) coordinate pair.

![image](https://github.com/user-attachments/assets/aa68424a-8c45-4b5e-b328-27315f4b5d38)

The shortest distance between both is a straight line. Given the two components of each vector, the SAD computes the sum of the length differences, which is a one-dimensional process. The SSD on the other hand, computes the sum of squares, which obeys the law of Pythagoras. This law says that in a rectangular triangle, the sum of the catheti squares is equal to the square of the hypotenuse. So in terms of the geometric distance between both vectors, the L2-norm is a more accurate measure. Note that the same rationale applies to higher-dimensional descriptors in the same manner.

In the case of a binary descriptor who consists only of ones and zeros, the best (and fastest) measure to use is the Hamming distance, which computes the difference between both vectors by using an XOR function, which returns zero if two bits are identical and one if two bits are different. So the sum of all XOR operations is simply the number of differing bits between both descriptors.

The key takeaway here is that you have to adapt the distance measure to the type of descriptor you are using. In case of gradient-based methods such as SIFT, the L2-norm would be most appropriate. In the case of all binary descriptors, the Hamming distance should be used.

### Brute Force Matching (AKA Nearest Neighbor Matching):
Let us assume we have N keypoints and their associated descriptors in one image and M keypoints in another image. The most obvious method to look for corresponding pairs would be to compare all features with each other, i.e. perform N x M comparisons. For a given keypoint from the first image, it takes every keypoint in the second image and calculates the distance. The keypoint with the smallest distance will be considered its pair. The output of brute force matching in OpenCV is a list of keypoint pairs sorted by the distance of their descriptors under the chosen distance function. Brute force matching works well for small keypoint numbers but can be computationally expensive as the number of keypoints increases. (available in OpenCV under the name BFMatcher)

### Fast Library for Approximate Nearest Neighbors (FLANN)
In 2014, David Lowe (the father of SIFT) and Marius Muja released an open-source library called "fast library for approximate nearest neighbors" (FLANN). FLANN trains an indexing structure for walking through potential matching candidates that is created using concepts from machine learning. The library builds a very efficient data structure (a KD-tree) to search for matching pairs and avoids the exhaustive search of the brute force approach. It is therefore faster while the results are still very good, depending on the matching parameters. (Available OpenCV lib)

