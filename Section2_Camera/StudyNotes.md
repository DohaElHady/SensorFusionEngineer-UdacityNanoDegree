# Concepts
- Intensity: Strength of the color in each pixel.
- Intensity Gradient: Rate of change of the intensity from one pixel to another in a certain direction.
  - Gradient is defined by 2 parameters: direction and magnitude.
    
## Image Noise Filtering
- **Box, mean or average filter**: average of the surrounding pixels, basic smoothing algorithm.
  - Example 3x3 box filter: <img width="100" height="60" alt="image" src="https://github.com/user-attachments/assets/6ae82b1c-3bb6-4b2b-bcc4-b3c3d25a55aa" />
- **Gaussian Filter**: used to smooth the image and filter out noise
  - Filter is shifted over the image and combined with the intensity values beneath it.
  - Filter has 2 parameters:
    1. The standard deviation, which controls the spatial extension of the filter in the image plane. The larger the standard deviation, the wider the area which is covered by the filter.
    2. The kernel size, which defines how many pixels around the center location will contribute to the smoothing operation.
  - Gaussian filter is a symmetric matrix around the center that is divided by the sum of all the elements inside the matrix for normalization.
  - Example 3x3 gaussian filter: <img width="100" height="60" alt="image" src="https://github.com/user-attachments/assets/8b4cbafb-9f93-4ecc-abcb-1d860579c321" />
  
- **Steps of Image Filtering**:
  1. Create a filter kernel with the desired properties (e.g. Gaussian smoothing)
  2. Define the anchor point within the kernel (usually the center position) and place it on top of the first pixel of the image.
  3. Compute the sum of the products of kernel coefficients with the corresponding image pixel values beneath.
  4. Place the result to the location of the kernel anchor in the input image.
  5. Repeat the process for all pixels over the entire image.
 
## Intensity Gradient
- **Sobel Operator**: used to calculate the intensity gradient per direction, it strips away image content and only leaves pixels which neighbourhood change quickly.
  - It is based on applying small integer-valued filters both in horizontal and vertical direction. 
  - The operators are 3x3 kernels, one for the gradient in x and one for the gradient in y.
    <p align="center"><img width="550" height="150" alt="image" src="https://github.com/user-attachments/assets/0ad55580-a0d5-4ea0-9117-a0f6409d67e4" /></p>
  - Its output is called "edge map".
    <p align="center"><img width="700" height="350" alt="image" src="https://github.com/user-attachments/assets/251bd022-b5ad-49b9-b011-9d2dec1a3492" /></p>
  - Based on the image gradients in both x and y, gradient magnitude and directions can be calculated:
    <p align="center"><img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/572a4603-b0bc-4dce-9737-19dbc24c10b6" /> <img width="400" height="322" alt="image" src="https://github.com/user-attachments/assets/46132502-36a5-46df-8a29-60d78a9821c6" /></p>
## Key Points Extraction
- **Harris Corner Detection**: 
