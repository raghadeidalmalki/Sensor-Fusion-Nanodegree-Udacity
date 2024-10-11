## Image Filters and Gaussian Smoothing

Before delving further into gradient computation, it's essential to consider noise, which is present in all images (except artificial ones) and tends to decrease with increasing light intensity. To mitigate noise, especially in low-light conditions, a smoothing operator should be applied to the image before computing the gradient. Typically, a Gaussian filter is used for this purpose, which is moved across the image and combined with the underlying intensity values. To properly configure the filter, two parameters need to be adjusted:

1.	The standard deviation, which controls the spatial extension of the filter in the image plane. The larger the standard deviation, the wider the area which is covered by the filter.
2.	The kernel size, which defines how many pixels around the center location will contribute to the smoothing operation.

The following figure shows three Gaussian filter kernels with varying standard deviations.

![image](https://github.com/user-attachments/assets/cc72c223-e004-46f4-a899-7a8b2f3bb560)


Gaussian smoothing operates by assigning each pixel a weighted sum of the surrounding pixels based on the height of the Gaussian curve at each point. The center pixel contributes the most, while the contribution from surrounding pixels decreases based on the Gaussian curve's height and standard deviation. As the standard deviation increases, the influence of neighboring pixels around the center also grows (as seen in the left image).

Applying the Gaussian filter (or any other filter) involves four successive steps, illustrated by the figure below:

1.	Create a filter kernel with the desired properties (e.g. Gaussian smoothing or edge detection)
2.	Define the anchor point within the kernel (usually the center position) and place it on top of the first pixel of the image.
3.	Compute the sum of the products of kernel coefficients with the corresponding image pixel values beneath.
4.	Place the result to the location of the kernel anchor in the input image.
5.	Repeat the process for all pixels over the entire image.
The following figure illustrates the process of shifting the (yellow) filter kernel over the image row by row and assigning the result of the two-dimensional sum H(x,y) to every pixel location. Note that x and y express a position in the coordinate system of the image while i and j are coordinates within the filter kernel. Also, ai and aj are the coordinates of the anchor point aa of the filter kernel. If you are unsure why both values need to be subtracted from x and y, try to insert the values for a simple 3x3 filter kernel by hand and see what the subtraction does.


![image](https://github.com/user-attachments/assets/34d35116-21ad-410c-8b24-27b4e5dc579a)

A filter kernel for Gaussian smoothing is shown in the next figure. The figure shows a discrete filter kernel with a central anchor point (at [4, 4]) corresponding to the maximum of the Gaussian curve and with decreasing values towards the edges in a (approximately) circular shape.


![image](https://github.com/user-attachments/assets/820a1345-9770-4291-98c4-c505877cbe4d)

#### Illustration of Applying Derivatives for Edge Detection

We apply the first and second derivatives to the intensity curve obtained by comparing the intensity values, resulting in a curve like the one given below:


![image](https://github.com/user-attachments/assets/69320a18-d19a-4a6f-afa6-c8a228478f93)

For the first derivative, the locations of edges are determined by their local extrema. For the second derivative, the locations of the edges are determined by their zero crossings.


