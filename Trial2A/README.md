Line follow progression
1. simple contour center
2. use lower and upper contours
3. average those contours to get a better guess
4. use a low pass filter to help reduce error
5. use canny edge detection and a blue mask to find the line
6. add a glare mask to reduce false positives
7. use a regression line to create a fit for the line and try to follow that
8. use a pure pursuit model using distance
9. change pure pursuit model to get the angle between the target point and the bottom center of the screen
10. use a bounding box around each contour to check if it touches two edges of the screen, as an actual line always will + add dotmatrix
