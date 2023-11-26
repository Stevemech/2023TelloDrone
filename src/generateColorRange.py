import cv2 
import numpy as np
  
cap = cv2.imread('yellow.png')
upper_r, upper_g, upper_b = 0, 0, 0
lower_r, lower_g, lower_b = 180, 180, 180
#You're free to do a resize or not, just for the example
cap = cv2.resize(cap, (340,480))
for x in range (0,340,1):
    for y in range(0,480,1):
        color = cap[y,x]
        upper_r = max(color[0], upper_r)
        upper_g = max(color[1], upper_g)
        upper_b = max(color[2], upper_b)
        lower_r = min(color[0], lower_r)
        lower_g = min(color[1], lower_g)
        lower_b = min(color[2], lower_b)
        #print(color)
print(upper_r, upper_g, upper_b)
print(lower_r, lower_g, lower_b)