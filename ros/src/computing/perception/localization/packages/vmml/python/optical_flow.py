import cv2
from copy import copy



def create_line (image1, image2, mask):
    
    orbDetector = cv2.ORB.create(6000)
    bfMatcher = cv2.BFMatcher.create(cv2.NORM_HAMMING2, crossCheck=True)
    
    keypoints1, descriptors1 = orbDetector.detectAndCompute(image1, mask)
    keypoints2, descriptors2 = orbDetector.detectAndCompute(image2, mask)
    
    matches = bfMatcher.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x:x.distance)
    
#     image3 = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches[:100], None, flags=2)
    
    image3 = cv2.cvtColor(copy(image2), cv2.COLOR_GRAY2BGR)
     
    colorGreen = (0,255,0)
    colorRed = (0,0,255)
    for i in range(500):
        m = matches[i]
         
        if (m.trainIdx >= len(keypoints1) or m.queryIdx >= len(keypoints2)):
            continue
         
        p1 = ( int(keypoints1[m.queryIdx].pt[0]), int(keypoints1[m.queryIdx].pt[1]) )
        p2 = ( int(keypoints2[m.trainIdx].pt[0]), int(keypoints2[m.trainIdx].pt[1]) )
         
        cv2.line(image3, p1, p2, colorGreen)
        cv2.circle(image3, p1, 1, colorRed)
    
    return image3


if __name__=='__main__' :
    
    sizei = (640,480)
    
    image1 = cv2.imread('/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/test/optical_flow/1.png', cv2.IMREAD_COLOR)
    image1 = cv2.resize(cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY), sizei)
    image2 = cv2.imread('/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/test/optical_flow/3.png', cv2.IMREAD_COLOR)
    image2 = cv2.resize(cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY), sizei)
    mask = cv2.imread('/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/test/meidai_mask.png', cv2.IMREAD_GRAYSCALE)
    mask = cv2.resize(mask, sizei)

    image3 = create_line(image1, image2, mask)
    cv2.imwrite('/tmp/x.png', image3)