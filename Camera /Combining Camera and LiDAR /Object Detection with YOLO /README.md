# Object Detection with YOLO 
Object detection methods primarily relied on histograms of oriented gradients (HOG) and support vector machines (SVM). Until the advent of deep-learning, this HOG/SVM approach was viewed as the leading technique in the field. Although it still yields satisfactory results for many applications, its effectiveness is constrained on platforms with limited processing power. Similar to SIFT, a significant drawback of this method is its dependence on intensity gradients, which can be computationally expensive.

Another approach to object detection involves deep learning frameworks like TensorFlow and Caffe. An easier alternative that is user-friendly and delivers immediate results is YOLO (You Only Look Once), a fast detection framework integrated into the OpenCV library. It **utilizes a single neural network that processes the entire image at once**. This network **divides the image into regions** and **predicts bounding boxes along with probabilities for each region**, weighting the bounding boxes by their predicted probabilities. The following figure illustrates this principle:

![image](https://github.com/user-attachments/assets/f677919e-680b-4fba-8ff3-ce142f64db02)

Unlike classifier-based systems like HOG/SVM, YOLO analyzes the entire image, allowing its predictions to be influenced by the overall context. It makes predictions in a single pass through the network, in contrast to methods like R-CNN, which require thousands of passes to process a single image. This efficiency contributes to YOLO's impressive speed while still producing results comparable to other advanced techniques, such as the Single Shot MultiBox Detector (SSD).

## YOLOv3 Workflow
- **Grid Division:** The input image is first divided into a 13x13 grid of cells. The size of these cells in pixels varies depending on the dimensions of the input image. For example, when using an image size of 416x416 pixels, each cell measures 32x32 pixels.

- **Bounding Box Prediction:** Each cell is responsible for predicting a set of bounding boxes. For each bounding box, the network estimates the confidence level that it contains a specific object, as well as the probability that the object belongs to a particular class (based on the COCO dataset).

- **Non-Maximum Suppression:** Finally, non-maximum suppression is applied to remove bounding boxes with low confidence scores and to eliminate redundant boxes that overlap significantly, ensuring that only the most relevant detections remain.

### Based on the code we have bellow the workflow goes as follows:


**Step 1: Initialize the Parameters**
Every bounding box predicted by YOLOv3 is associated with a confidence score. The parameter 'confThreshold' is used to remove all bounding boxes with a lower score value.

Then, a non-maximum suppression is applied to the remaining bounding boxes. The NMS procedure is controlled by the parameter ‘nmsThreshold‘.

The size of the input image is controlled by the parameters ‘inpWidth‘ and ‘inpHeight‘, which is set to 416 as proposed by the YOLO authors. Other values could e.g. be 320 (faster) or 608 (more accurate).

**Step 2: Prepare the Model**
The file 'yolov3.weights' contains the **pre-trained network’s weights** and has been made available by the authors of YOLO

The file 'yolov3.cfg' containing the **network configuration** is available for download, and the coco.names file which contains the 80 different **class names** used in the COCO dataset can be downloaded.

The following code shows how to load the model weights as well as the associated model configuration:

``` ruby
 // load image from file
    cv::Mat img = cv::imread("./images/img1.png");

    // load class names from file
    string yoloBasePath = "./dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights"; 

    vector<string> classes;
    ifstream ifs(yoloClassesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    // load neural network
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
```
***After loading the network, the DNN backend is set to DNN_BACKEND_OPENCV. If OpenCV is built with Intel’s Inference Engine, DNN_BACKEND_INFERENCE_ENGINE should be used instead. The target is set to CPU in the code, as opposed to using DNN_TARGET_OPENCL, which would be the method of choice if a (Intel) GPU was available.***

**Step 3: Generate 4D Blob from Input Image**
As data flows through the network, YOLO stores, communicates, and manipulates the information as **"blobs":** the blob is the standard array and unified memory interface for many frameworks, including Caffe. A blob is a wrapper over the actual data being processed and passed along and also provides synchronization capability between the CPU and the GPU. Mathematically, a blob is an N-dimensional array stored in a C-contiguous fashion. The conventional blob dimensions for batches of image data are number N x channel C x height H x width W. In this nomenclature, N is the batch size of the data. Batch processing achieves better throughput for communication and device processing. For a training batch of 256 images, N would be 256. The parameter C represents the feature dimension, e.g. for RGB images C = 3. In OpenCV, blobs are stored as 4-dimensional ```cv::Mat``` array with NCHW dimensions order. More details on blobs can be found [here](https://caffe.berkeleyvision.org/tutorial/net_layer_blob.html)

The following example illustrates the memory structure of a blob with N=2, C=16 channels and height H=5 / width W=4.

![image](https://github.com/user-attachments/assets/90dbe0a5-ff7a-4401-98da-ab91d9d561cc)

The code below shows how an image loaded from the file is passed through the blobFromImage function to be converted into an input block for the neural network. The pixel values are scaled with a scaling factor of 1/255 to a target range of 0 to 1. It also adjusts the size of the image to the specified size of (416, 416, 416) without cropping.

```ruby
 // generate 4D blob from input image
    cv::Mat blob;
    double scalefactor = 1/255.0;
    cv::Size size = cv::Size(416, 416);
    cv::Scalar mean = cv::Scalar(0,0,0);
    bool swapRB = false;
    bool crop = false;
    cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
```
Later in the code, the output blob is passed as input to the network, followed by a forward pass to generate a list of predicted bounding boxes. These bounding boxes then undergo a post-processing step to filter out those with low confidence scores. Let's examine these steps in more detail.


Next, we need to pass the blob we just created to the network as input. Then, we execute the forward function in OpenCV to perform a single forward pass through the network. To do this, we must identify the last layer of the network and provide its corresponding internal names to the function. This can be accomplished using the OpenCV function ```getUnconnectedOutLayers```, which retrieves the names of all unconnected output layers, effectively representing the last layers of the network. The following code demonstrates how to achieve this:

```ruby
 // Get names of output layers
    vector<cv::String> names;
    vector<int> outLayers = net.getUnconnectedOutLayers(); // get indices of output layers, i.e. layers with unconnected outputs
    vector<cv::String> layersNames = net.getLayerNames(); // get names of all layers in the network

    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) // Get the names of the output layers in names
    {
        names[i] = layersNames[outLayers[i] - 1];
    }

    // invoke forward propagation through network
    vector<cv::Mat> netOutput;
    net.setInput(blob);
    net.forward(netOutput, names);
```

The result of the forward pass and thus the network's output, is a vector of size C (where C represents the number of classes in the blob). The first four elements for each class indicate the x and y coordinates of the bounding box center, along with its width and height. The fifth element reflects the confidence that the bounding box contains an object. The remaining elements in the vector represent the confidence scores for each class listed in the coco.cfg file. In subsequent steps of the code, each bounding box is assigned to the class with the highest confidence score.

The following code shows how to scan through the network results and assemble the bounding boxes with a sufficiently high confidence score into a vector. The function ```cv::minMaxLoc``` finds the minimum and maximum element values and their positions with extremes searched across the whole array.

```ruby
// Scan through all bounding boxes and keep only the ones with high confidence
    float confThreshold = 0.20;
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;
    for (size_t i = 0; i < netOutput.size(); ++i)
    {
        float *data = (float*)netOutput[i].data;
        for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
        {
            cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
            cv::Point classId;
            double confidence;

            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
            if (confidence > confThreshold)
            {
                cv::Rect box; int cx, cy;
                cx = (int)(data[0] * img.cols);
                cy = (int)(data[1] * img.rows);
                box.width = (int)(data[2] * img.cols);
                box.height = (int)(data[3] * img.rows);
                box.x = cx - box.width/2; // left
                box.y = cy - box.height/2; // top

                boxes.push_back(box);
                classIds.push_back(classId.x);
                confidences.push_back((float)confidence);
            }
        }
    }
```

Applying the YOLOv3 algorithm to our highway image provides the following result:

![image](https://github.com/user-attachments/assets/b2bd5f22-206f-458d-8d0e-e131f3055334)

As can be seen, the car in the left lane is covered by two bounding boxes of similar size. To avoid the occurrence of redundant boxes, the last step performs a **non-maximum suppression** which aims at keeping only the bounding box with the highest confidence score.

**Step 5: Post-Processing of Network Output**
The OpenCV library offers a ready-made function for the suppression of overlapping bounding boxes. This function is called NMSBoxes and it can be used as illustrated by the following short code sample:

```ruby
// perform non-maxima suppression
    float nmsThreshold = 0.4;  // Non-maximum suppression threshold
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
```

After applying non-maximum suppression, redundant bounding boxes will have been successfully removed. The following figure shows the results, where green indicates preserved bounding boxes while red bounding boxes have been removed during NMS.

![image](https://github.com/user-attachments/assets/165f148d-9129-49d7-9002-fdbc4f6a7741)






