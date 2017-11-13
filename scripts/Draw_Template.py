'''TO draw the virtual template for the AH'''

import numpy as np
import cv2

# BGR mode in python
#Black image
img = np.zeros((600,300), np.uint8) #screen size i.e. 600 x 300 pix
img[:]= 255
cv2.line(img,(0,300),(300,300),0,2) # file,start,end,color,thickness
cv2.rectangle(img, (0,0),(300,600),0,5) # top-left and right-bottom
cv2.rectangle(img, (100,0),(200,10),0,2)
cv2.rectangle(img, (100,590),(200,600),0,2)
cv2.circle(img,(150,300),75,0, 2) # centre co-ordinates , radius
cv2.imwrite("Template_Virtual_AH.jpg",img)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
