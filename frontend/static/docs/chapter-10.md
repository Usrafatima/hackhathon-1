---
title: 'Chapter 10: Computer Vision for Robotics'
metadata:
  chapter_number: 10
  keywords: ['computer vision', 'robot vision', 'image processing', 'deep learning', 'cnn', 'yolo', 'object detection', 'segmentation']
---

# Chapter 10: Computer Vision for Robotics


## Goals


- **To introduce the role of computer vision as a primary sensing modality for robots.** We will explore why "seeing" is so critical for intelligent interaction with the world.
- **To cover fundamental image processing techniques** that form the basis of classical computer vision.
- **To provide an overview of modern deep learning-based approaches** for crucial tasks like object detection and segmentation.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Describe the main stages of a typical computer vision pipeline.**
- **Explain the purpose of image filtering and feature detection** in classical computer vision.
- **Differentiate between object detection, semantic segmentation, and instance segmentation,** and identify the output of each.

## Full Explanations of Concepts



### Introduction to Robot Vision

**Why is vision so important?** While sensors like LiDAR are excellent for measuring geometry ("there is an object 2.5 meters away"), cameras provide much richer information. Vision allows a robot to understand the *semantics* of a scene.
- **Richness:** Cameras provide color, texture, shape, and fine-grained detail that is unavailable to most other sensors.
- **Cost:** Cameras are inexpensive, ubiquitous, and power-efficient.
- **Functionality:** Vision is the only way to perform tasks like reading text, recognizing faces, or identifying specific object types (e.g., distinguishing between a coffee mug and a water glass).

**The Challenge:** The richness of vision is also its biggest challenge. A single image is a massive array of pixel values. Deriving high-level semantic understanding ("that's a dog chasing a ball") from this low-level data is an incredibly complex computational problem that the human brain is highly optimized for, but which has taken decades for computers to begin to master.

### The Classical Computer Vision Pipeline

Before the deep learning revolution, computer vision systems were built on a pipeline of "hand-engineered" steps. Understanding this pipeline provides crucial context.

1.  **Image Acquisition:** Capturing the image from a camera and transferring it to the robot's processor.
2.  **Preprocessing:** Cleaning up the image to make it easier to process.
    - **Color Space Conversion:** Many algorithms don't need color information. Converting an RGB image to **grayscale** simplifies the data from a 3D array (Height x Width x 3) to a 2D array (Height x Width x 1), making subsequent steps much faster.
    - **Filtering/Smoothing:** Images are often noisy. Applying a filter, such as a **Gaussian blur**, averages pixel values with their neighbors, smoothing out random noise.
3.  **Feature Extraction:** Identifying "interesting" parts of the image. Instead of looking at millions of pixels, the robot can focus on a few thousand key features.
    - **Edges:** Finding pixels that represent a sharp change in intensity. The **Canny edge detector** is a classic multi-stage algorithm for producing clean, thin edges.
    - **Corners:** Finding points where edges in different directions meet. The **Harris corner detector** is a well-known algorithm for this.
    - **Feature Descriptors:** Once a keypoint (like a corner) is found, a "descriptor" is computed for the patch of pixels around it. This descriptor is a vector of numbers that acts as a unique "fingerprint" for that feature. Famous examples include **SIFT** (Scale-Invariant Feature Transform), **SURF**, and **ORB**. The key property of these descriptors is that they are robust to changes in scale, rotation, and lighting, allowing the robot to recognize the same feature from different viewpoints.
4.  **Recognition/Decision-Making:** Using the extracted features to perform a task. For example, to find a specific book, the robot could compare the ORB features from its live camera feed to a set of pre-computed ORB features from a photo of the book's cover.

### The Deep Learning Revolution: Convolutional Neural Networks (CNNs)

The major limitation of the classical pipeline was that humans had to manually design the feature extractors. A breakthrough came with **Convolutional Neural Networks (CNNs)**, a type of deep learning model that *learns* the features automatically from data.

- **How it Works:** A CNN consists of many layers. The initial layers automatically learn to recognize very basic features like edges and colors. Subsequent layers combine these simple features to learn more complex ones, like textures and patterns. Deeper layers combine those to recognize object parts (an eye, a nose, a wheel), and the final layers combine those parts to recognize entire objects.
- **The Shift:** Instead of a human programmer deciding that "a face has two eyes and a mouth," the network learns this hierarchy of features on its own by being shown millions of example images. This led to a massive leap in performance on vision tasks.

### Modern Computer Vision Tasks for Robotics

Using CNNs, robots can now perform a variety of sophisticated perception tasks.

#### 1. Image Classification
- **Question:** "What is the primary subject of this image?"
- **Output:** A single label for the entire image (e.g., "cat," "dog," "car").
- **Limitation:** It doesn't tell you *where* the object is or if there are multiple objects.

#### 2. Object Detection
- **Question:** "What objects are in this image, and where are they?"
- **Output:** A list of detections. Each detection consists of a **bounding box** (the coordinates of a rectangle surrounding the object), a class label (e.g., "person"), and a confidence score (e.g., 95%).
- **Architectures:**
    - **Two-Stage Detectors (e.g., Faster R-CNN):** These methods first propose a set of "regions of interest" where an object might be, and then run a classifier on each region. They are very accurate but often slower.
    - **One-Stage Detectors (e.g., YOLO - You Only Look Once, SSD):** These methods cleverly predict the bounding boxes and class probabilities in a single pass of the network. They are extremely fast and are the preferred choice for real-time robotics applications.

#### 3. Semantic Segmentation
- **Question:** "What category does each pixel in this image belong to?"
- **Output:** A "mask" the same size as the input image, where each pixel is color-coded according to its class. For example, all "road" pixels might be colored purple, all "sky" pixels blue, and all "car" pixels red.
- **Limitation:** It does not distinguish between different instances of the same class. If there are two cars next to each other, they will both be colored the same shade of red.

#### 4. Instance Segmentation
- **Question:** "Which object instance does each pixel belong to?"
- **Output:** The ultimate level of scene understanding. It provides a mask where every object instance is individually identified.
- **Example:** In an image with two cars, the road is green, the sky is blue. The pixels for the first dog are colored red. The pixels for the second dog are colored pink.
- **Famous Architecture:** **Mask R-CNN** is a well-known model that extends Faster R-CNN to predict a segmentation mask for each bounding box it finds.

## Step-by-Step Diagram Explanation

### The Evolution of Computer Vision Tasks

![Evolution of Vision Tasks](https://i.imgur.com/example-diagram.png)
*(Note: Replace with the 4-panel diagram described)*

This diagram shows the output of the four main vision tasks on a single input image containing two dogs on a grassy field.

1.  **Panel 1: Classification**
    - **Input:** Image of two dogs in a field.
    - **Output:** A single text label below the image: `"dog"`.

2.  **Panel 2: Object Detection**
    - **Input:** Same image.
    - **Output:** The image now has two rectangular bounding boxes drawn on it. One box is tightly drawn around the first dog, and the other is drawn around the second. Each box has a small label "dog: 98%" next to it.

3.  **Panel 3: Semantic Segmentation**
    - **Input:** Same image.
    - **Output:** The image has been completely "colorized." Every pixel belonging to a dog is colored red. Every pixel belonging to the grass is colored green. The sky pixels are colored blue. There is no distinction between the two dogs; they are both part of the red "dog" class.

4.  **Panel 4: Instance Segmentation**
    - **Input:** Same image.
    - **Output:** The image is again colorized, but with more detail. The grass is green, the sky is blue. The pixels for the first dog are colored red. The pixels for the second dog are colored pink. Each object *instance* is segmented individually.

## Lab Instructions



### Lab 10.1: Real-Time Object Detection with YOLO and ROS

**Reasoning:**
This lab provides direct, hands-on experience with a state-of-the-art deep learning model. It demonstrates how to integrate a powerful, pre-trained network into a ROS system to give a robot the ability to "see" and identify objects in real-time.

**Instructions:**
1.  **Prerequisites:** A computer with ROS and Python. A standard USB webcam. An NVIDIA GPU with CUDA is highly recommended for real-time performance, but it will also run (more slowly) on a CPU.
2.  **Install YOLOv5:** YOLO (You Only Look Once) is a very popular and fast object detection model. We will use the v5 implementation from Ultralytics.
    ```bash
    # Clone the repository
    git clone https://github.com/ultralytics/yolov5
    cd yolov5
    # Install dependencies
    pip install -r requirements.txt
    ```
3.  **Test YOLOv5:** The repository comes with pre-trained weights (`yolov5s.pt` is the small, fast version) and test images. Run the detector to confirm it works.
    ```bash
    python detect.py --weights yolov5s.pt --source data/images/bus.jpg
    ```
    Check the `runs/detect/exp` folder. You should find a copy of `bus.jpg` with bounding boxes drawn on it.
4.  **Setup ROS Webcam Node:**
    - Install the ROS package for webcams: `sudo apt-get install ros-<distro>-usb-cam`.
    - In a terminal, run the camera node: `rosrun usb_cam usb_cam_node`.
    - In another terminal, verify that the image topic is being published: `rostopic echo /usb_cam/image_raw`. You should see a stream of data.
5.  **Create the ROS YOLO Node:**
    - Create a new Python script in your ROS package.
    - **Key Libraries:** You will need `rospy`, `cv2` (from OpenCV), and `cv_bridge` to convert between ROS images and OpenCV images. You'll also need to import `torch`.
    - **Structure of the Node:**
        - Initialize your ROS node.
        - Load the YOLOv5 model *once* at the beginning: `model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)`.
        - Create an image subscriber that listens to `/usb_cam/image_raw`.
        - Create a callback function `image_callback(msg)` that will process each incoming image.
    - **Inside the Callback:**
        - Use `CvBridge` to convert the ROS `sensor_msgs/Image` (`msg`) into an OpenCV image (a NumPy array).
        - The YOLOv5 model expects RGB images, but OpenCV/ROS often use BGR, so you may need to convert the color channels.
        - Feed the image into the model: `results = model(cv_image)`.
        - The `results` object contains the detections. You can loop through them to get the bounding box coordinates (`xyxy`), confidence scores, and class names.
        - Use OpenCV functions like `cv2.rectangle()` and `cv2.putText()` to draw these results onto the `cv_image`.
        - Use `cv2.imshow("YOLO Detections", cv_image)` to display the final annotated image in a window. Remember to include `cv2.waitKey(1)`.
6.  **Run:**
    - In one terminal, run `roscore`.
    - In another, run the `usb_cam` node.
    - In a third, run your YOLO detector ROS node.
    - **Observe:** A window should appear showing your live webcam feed. Point it at various objects (a person, a cup, a cell phone, a banana). You should see bounding boxes appear in real-time, correctly identifying the objects.

**Expected Outcome:**
- Students will have built an incredibly powerful perception system with relatively little code by leveraging pre-trained models.
- They will understand the practical workflow of a ROS perception node: **Subscribe → Convert (cv_bridge) → Process (YOLO) → Visualize (OpenCV)**.
- This provides a concrete example of how modern AI can be seamlessly integrated into a modular robotics framework like ROS.
