import cv2
import numpy as np

print 'opening Hyundai-Verna.webp'
img = cv2.imread('./images/Hyundai-Verna.webp', cv2.IMREAD_GRAYSCALE)
print img
cv2.imshow('verna', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

print 'saving gray image'
cv2.imwrite('./images/gray-hyundai-verna.png', img)
