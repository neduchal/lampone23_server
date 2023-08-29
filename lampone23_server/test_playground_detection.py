import cv2
import numpy as np


def normalize_img(img, mask_size=5):
    blur = cv2.GaussianBlur(img,(5,5),0)
    ret3,mask = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)*255    
    #mask = cv2.filter2D(img, -1, np.ones((mask_size,mask_size))/(mask_size**2), borderType=cv2.BORDER_CONSTANT)
    
    # Create a mask with blured image with the same value range
    #mask =scipy.signal.convolve2d(img, np.ones((mask_size,mask_size))/(mask_size**2),mode='same',boundary='symm')

    # Substract and normalize to <0,1> range
    #normalized = img-mask+0.5
    return normalized

def nms(input, neighborhood_size=40):
    # Add padding with size equal to neighborhood size (so we dont lose information from the image edges)
    padding = neighborhood_size
    img_padded = np.pad(input, padding, mode="symmetric")

    # Prepare the result array
    result = np.zeros(input.shape)

    # Iterate through the image
    for i in range(input.shape[0]):
        i_ = i+padding
        for j in range(input.shape[1]):
            j_ = j+padding

        # Find maximum in the neighborhood
        max_val = np.max(img_padded[i_-neighborhood_size:i_+neighborhood_size,j_-neighborhood_size:j_+neighborhood_size])

        # Make the output array
        if max_val == img_padded[i_,j_]:
            result[i,j] = 1
    return result.T

def get_grid(img):
    # Normalize image
    normalized = normalize_img(img)

    # Creating kernel with pattern
    kernel = np.ones((9,9))
    kernel[3:-3,:] = 0
    kernel[:,3:-3] = 0
    print('Kernel:')
    print(kernel)

    # Convolution with kernel
    convolution_output = cv2.filter2D(normalized, -1, kernel, borderType=cv2.BORDER_CONSTANT)
    #convolution_output =scipy.signal.convolve2d(normalized, kernel, mode='same')
    cv2.imwrite("/var/www/html/image/norm.jpg", convolution_output)


    # Finding the local maximums
    points = nms(convolution_output)
    
    print('Number of detected points:')
    print(np.sum(points))

    return np.where(points)



frame = cv2.imread("/var/www/html/image/image.png")


image = frame[260:761, 652:1328, :]

cv2.imwrite("/var/www/html/image/test.jpg", image)

img_gray = cv2.cvtColor(image[:,:,:3], cv2.COLOR_BGR2GRAY)
points = get_grid(img_gray)

image2 = image.copy()

for point in points:
    image2 = cv2.circle(image2, (point[0], point[1]), 5, (0,255,0), 3)

cv2.imwrite("/var/www/html/image/corners.jpg", image)
