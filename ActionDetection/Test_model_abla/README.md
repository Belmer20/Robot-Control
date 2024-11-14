# RT_GestureRecognition
This demo was trained on The 20BN-jester Dataset V1 (https://www.kaggle.com/datasets/toxicmender/20bn-jester?fbclid=IwAR3PEWLcJkOFqxvhBAt89HxiYe8JsWLcClUsOVEH1QC6nbZmb5rLt51R4Ng)


# Getting the demo to work:

Clone the repository.
You might need to install some libs such as:
  - PyTorch
  - OpenCV2
  - Numpy
  - Pandas 
  - Torchvision
  
Run `demo.py`

Press `Q` to stop.

# What is this network?

This is a custom NN using 2D and 3D CNN layers to achieve online video recognition. 
The model is trained on the Jester dataset, and adapted to the other dataset via trasnfer learning.

# Getting the datasets used

The data used to train and test can be downloaded in the following links:

- Jester: https://www.kaggle.com/datasets/toxicmender/20bn-jester?fbclid=IwAR3PEWLcJkOFqxvhBAt89HxiYe8JsWLcClUsOVEH1QC6nbZmb5rLt51R4Ng


# Demo:

- You need to run the script in the demo.py file for GPU acceleration use the code in this file demo_GPU.py



