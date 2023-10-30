#include "laserprocessing.h" 
#include <algorithm>
#include <numeric>
#include <fstream> // header for saving images

using namespace std;
using namespace cv;

// Constructor assigns values to local variables when an object of this class is called
LaserProcessing::LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth, sensor_msgs::CameraInfo caminfo, nav_msgs::Odometry odo):
    image_(image), depth_(depth), squareDetected_(false), camInfo_(caminfo), odo_(odo)
{
}

void LaserProcessing::setSquareDetected(bool squareDetected) {
    squareDetected_ = squareDetected; //Assigns the value of squareDetected to the local variable
}

// The following function returns whether or not a square has been detected, by calling functions that perform Harris Corner Detection
bool LaserProcessing::testMessages(double& y_int, double& turnAngle, double& distance) {
    convertToGreyscale();
    if(squareDetected_) {
        // normalAngleToBot(); // Finds the relative angle between the square's normal and Turtlebot
        // findNormal(); // Finds the equation of the square's normal
        turnAngle = angleParallel_; // Sets the angle the Turtlebot needs to turn to face parallel to the square, passes back by reference

        y_int = y_int_; // Sets the y-intercept of the equation of the normal, passes back by reference
        distance = distance_; // Sets the perpendicular distance between the Turtlebot and the normal, passes back by reference
    }
    return squareDetected_; // Returns whether the square has been detected or not
}

// The following function finds the centre of the detected square, calculates the distance to the square and the relative angle to the square's angle, passes back by reference
void LaserProcessing::findCentre(double& depth1, double& turnAngle1) {
    convertToGreyscale();
    double depth = (topLeft3D_.x + topRight3D_.x)/2;
    std::cout << "depth: " << depth << std::endl;
    double turnAngle = 0;
    if(squareDetected_) {
        double centreY = (topLeft3D_.y + topRight3D_.y)/2;
        std::cout << "centre: " << centreY << std::endl;
        if(centreY>0.05) { // Square is to the left of the camera centre
            turnAngle = -atan(abs(centreY)/depth);
        }
        else if(centreY<-0.05) { // Square is to the right of the camera centre
            turnAngle = atan(abs(centreY)/depth);
        }
    }
    depth1 = depth;
    turnAngle1 = turnAngle;
    std::cout << "turn angle: " << turnAngle << std::endl;
}

// The following function converts the image from the RGB camera into greyscale, calls the Harris Corner Detection and saving functions
void LaserProcessing::convertToGreyscale() {
    std::string imageEncoding = image_.encoding;

    // Check if the image is already grayscale (mono8)
    if (imageEncoding == "mono8") {
        ROS_INFO("Image is already grayscale (mono8). No conversion needed.");
        return; // No need to convert
    }
    else if (imageEncoding == "rgb8"){
        // Get image dimensions
        int width = image_.width;
        int height = image_.height;

        // Initialize the grayscale image
        greyscale_image_.resize(width * height);

        // Convert the RGB image to grayscale
        for (int yPixel = 0; yPixel < height; yPixel++){
            for (int xPixel = 0; xPixel < width; xPixel++){
                // Calculate the grayscale value using a weighted average
                int r_index = yPixel * width * 3 + xPixel * 3; 
                int g_index = r_index + 1;
                int b_index = r_index + 2;

                //Find the value of the pixel in RGB
                uint8_t r = image_.data[r_index];
                uint8_t g = image_.data[g_index];
                uint8_t b = image_.data[b_index];

                //Calculate the greyscale value
                uint8_t greyscale_value = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
                
                // Set the grayscale value in the image data
                greyscale_image_[yPixel * width + xPixel] = greyscale_value;
            }
        }
        src_gray = Mat(height, width, CV_8UC1, greyscale_image_.data()); // Converts greyscale image into a matrix
        namedWindow( source_window, WINDOW_AUTOSIZE );
        createTrackbar("Threshold:", source_window, &thresh, max_thresh, staticCornerHarris, this);
        imshow( source_window, src_gray );
        cornerHarris_demo( 0, 0 ); // Calls the Harris Corner Detection function

        // Set the flag to indicate that the grayscale image is initialised
        greyscale_image_initialised_ = true;
        if (greyscale_image_initialised_){
            saveGreyscaleAsPGM("grayscale_image.pgm", greyscale_image_, width, height); // Save greyscale image by calling function
        }
        return;
    }
    return;
}

// The following function saves the greyscale image as a PGM file in the catkin_ws/src folder
void LaserProcessing::saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height)
{
    std::ofstream outfile(filename.c_str(), std::ios::binary);
    if (!outfile)
    {
        std::cerr << "Error: Unable to open the output file." << std::endl;
        return;
    }

    // Write PGM header
    outfile << "P5\n"; // P5 indicates binary PGM
    outfile << width << " " << height << "\n"; // Image dimensions
    outfile << "255\n"; // Maximum pixel value (8-bit grayscale)

    // Write image data
    outfile.write(reinterpret_cast<const char*>(grayscale_image.data()), grayscale_image.size()); // Overwrite the previous file

    outfile.close();

    std::cout << "Grayscale image saved as " << filename << std::endl;
}

// The following function uses an OpenCV function to detect corners in the image using Harris Corner Detection algorithm
void LaserProcessing::cornerHarris_demo(int, void*) {
    setSquareDetected(false);
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros(src_gray.size(), CV_32FC1);
    int blockSize = 3;
    int apertureSize = 3;
    double k = 0.04;
    cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    corner_x_coords.clear();
    corner_y_coords.clear();
    if((int) dst_norm.at<float>(1,1) < thresh) {
        for(int j = 0; j < dst_norm.rows ; j++) {
            for(int i = 0; i < dst_norm.cols; i++) {
                if((int) dst_norm.at<float>(j,i) > thresh) {
                    // Print the corner location (coordinates)
                    std::cout << "Corner detected at (" << i << ", " << j << ")" << std::endl;
                    corner_x_coords.push_back(i);
                    corner_y_coords.push_back(j);

                    circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
                }
            }
        }
    }

    if(corner_x_coords.size()>3 && corner_x_coords.size()<25) { // A square is detected if the number of corners detected is within these boundaries
        setSquareDetected(true);

        // The following stores all the detected corner pixel coordinates in a vector
        squareCornersX_.clear();
        squareCornersY_.clear();
        squareCornersX_.push_back(corner_x_coords.at(0));
        squareCornersY_.push_back(corner_y_coords.at(0));

        // The following for loop filters out pixel coordinates that are too close (i.e. refers to the same corner)
        for(int i=1; i<corner_x_coords.size(); i++) {
            bool same = false;
            for(int j=0; j<squareCornersX_.size(); j++) {
                if(abs(squareCornersX_.at(j)-corner_x_coords.at(i))<3 && abs(squareCornersY_.at(j)-corner_y_coords.at(i))<3) {
                    same = true;
                    break;
                }
            }
            if(!same) {
                squareCornersX_.push_back(corner_x_coords.at(i));
                squareCornersY_.push_back(corner_y_coords.at(i));
            }
            if(squareCornersX_.size() == 4) break; // If four distinct corners have been detected, break out of thr for loop
        }
        
        // If less than four corners are detected, then no square is detected
        if(squareCornersX_.size() < 4) setSquareDetected(false);
        // Otherwise print the coordinates to terminal (for debugging)
        else {
            std::cout << "x1,y1: " << squareCornersX_.at(0) << ", " << squareCornersY_.at(0) << std::endl;
            std::cout << "x2,y2: " << squareCornersX_.at(1) << ", " << squareCornersY_.at(1) << std::endl;
            std::cout << "x3,y3: " << squareCornersX_.at(2) << ", " << squareCornersY_.at(2) << std::endl;
            std::cout << "x4,y4: " << squareCornersX_.at(3) << ", " << squareCornersY_.at(3) << std::endl;

            // The following section finds the centre of the square by taking the average of all x and y coordinates
            int sumX = 0;
            int sumY = 0;
            for(int i=0; i<4; i++) {
                sumX += squareCornersX_.at(i);
                sumY += squareCornersY_.at(i);
            }
            centreX_ = sumX/4;
            centreY_ = sumY/4;
            
            // The following section assigns coordinates to each corner of the square based on their position relative to the square centre
            for(int i=0; i<4; i++) {
                if(squareCornersX_.at(i) < sumX/4 && squareCornersY_.at(i) < sumY/4) {
                    topLeft_.x = squareCornersX_.at(i);
                    topLeft_.y = squareCornersY_.at(i);
                }
                else if(squareCornersX_.at(i) > sumX/4 && squareCornersY_.at(i) < sumY/4) {
                    topRight_.x = squareCornersX_.at(i);
                    topRight_.y = squareCornersY_.at(i);
                }
                else if(squareCornersX_.at(i) < sumX/4 && squareCornersY_.at(i) > sumY/4) {
                    bottomLeft_.x = squareCornersX_.at(i);
                    bottomLeft_.y = squareCornersY_.at(i);
                }
                else if(squareCornersX_.at(i) > sumX/4 && squareCornersY_.at(i) > sumY/4) {
                    bottomRight_.x = squareCornersX_.at(i);
                    bottomRight_.y = squareCornersY_.at(i);
                }
            }

            // Print out the new 2D coordinates (for debugging)
            std::cout << "top left: " << topLeft_.x << ", " << topLeft_.y << std::endl;
            std::cout << "top right: " << topRight_.x << ", " << topRight_.y << std::endl;
            std::cout << "bottom left: " << bottomLeft_.x << ", " << bottomLeft_.y << std::endl;
            std::cout << "bottom right: " << bottomRight_.x << ", " << bottomRight_.y << std::endl;

            calculate3DCoords(); // Calls function to convert the 2D coordinates to both 3D local and 3D global coordinates
        }
    }

}

// The following function assists with OpenCV Harris Corner Detection
void LaserProcessing::staticCornerHarris(int value, void* userdata) {
        LaserProcessing* instance = reinterpret_cast<LaserProcessing*>(userdata);
        instance->cornerHarris_demo(value, userdata);
    }

// The following function convert the 2D coordinates to both 3D local and 3D global coordinates
void LaserProcessing::calculate3DCoords() {
    //principle points (from RGB camera intrinsic parameters)
    double cx = 960.5;
    double cy = 540.5;
    //focal lengths (from RGB camera intrinsic parameters)
    double fx = 1206.8897719532354;
    double fy = 1206.8897719532354;
    //square size in the real world (metres)
    double d = 0.5;

    //topLeft3D (local)
        topLeft3D_.y = (topLeft_.x-cx)*((d*fy)/(bottomLeft_.y-topLeft_.y))*(1/fx);
        topLeft3D_.z = (topLeft_.y-cy)*((d*fy)/(topLeft_.y-bottomLeft_.y))*(1/fy);
        topLeft3D_.x = ((d*fy)/(bottomLeft_.y-topLeft_.y));
    //topRight3D (local)
        topRight3D_.y = (topRight_.x-cx)*((d*fy)/(bottomRight_.y-topRight_.y))*(1/fx);
        topRight3D_.z = (topRight_.y-cy)*((d*fy)/(topRight_.y-bottomRight_.y))*(1/fy);
        topRight3D_.x = ((d*fy)/(bottomRight_.y-topRight_.y));
    //bottomLeft3D (local)
        bottomLeft3D_.y = (bottomLeft_.x-cx)*((d*fy)/(bottomLeft_.y-topLeft_.y))*(1/fx);
        bottomLeft3D_.z = (bottomLeft_.y-cy)*((d*fy)/(topLeft_.y-bottomLeft_.y))*(1/fy);
        bottomLeft3D_.x = ((d*fy)/(bottomLeft_.y-topLeft_.y));
    //bottomRight3D (local)
        bottomRight3D_.y = (bottomRight_.x-cx)*((d*fy)/(bottomRight_.y-topRight_.y))*(1/fx);
        bottomRight3D_.z = (bottomRight_.y-cy)*((d*fy)/(topRight_.y-bottomRight_.y))*(1/fy);
        bottomRight3D_.x = ((d*fy)/(bottomRight_.y-topRight_.y));
    // Print to terminal (for debugging)
    std::cout << "topLeft3D: " << topLeft3D_.x <<", " << topLeft3D_.y << ", " << topLeft3D_.z << std::endl;
    std::cout << "topRight3D: " << topRight3D_.x <<", " << topRight3D_.y << ", " << topRight3D_.z << std::endl;
    std::cout << "bottomLeft3D: " << bottomLeft3D_.x <<", " << bottomLeft3D_.y << ", " << bottomLeft3D_.z << std::endl;
    std::cout << "bottomRight3D: " << bottomRight3D_.x <<", " << bottomRight3D_.y << ", " << bottomRight3D_.z << std::endl;

    // The following section converts each 3D local coordinate into global coordinates
    topLeft3DGlobal_ = LaserProcessing::localToGlobal(topLeft3D_);
    topRight3DGlobal_ = LaserProcessing::localToGlobal(topRight3D_);
    bottomLeft3DGlobal_ = LaserProcessing::localToGlobal(bottomLeft3D_);
    bottomRight3DGlobal_ = LaserProcessing::localToGlobal(bottomRight3D_);
    // Print to terminal (for debugging)
    std::cout << "topLeft3D global: " << topLeft3DGlobal_.x <<", " << topLeft3DGlobal_.y << ", " << topLeft3DGlobal_.z << std::endl;
    std::cout << "topRight3D global: " << topRight3DGlobal_.x <<", " << topRight3DGlobal_.y << ", " << topRight3DGlobal_.z << std::endl;
    std::cout << "bottomLeft3D global: " << bottomLeft3DGlobal_.x <<", " << bottomLeft3DGlobal_.y << ", " << bottomLeft3DGlobal_.z << std::endl;
    std::cout << "bottomRight3D global: " << bottomRight3DGlobal_.x <<", " << bottomRight3DGlobal_.y << ", " << bottomRight3DGlobal_.z << std::endl;

    // The following section calculates centre of the square using global coordinates, ignoring its distance from Turtlebot
    centreGlobal_.x = (bottomRight3DGlobal_.x + bottomLeft3DGlobal_.x)/2;
    centreGlobal_.y = (bottomRight3DGlobal_.y + bottomLeft3DGlobal_.y)/2;
    centreGlobal_.z = 0;
}

// The following function calculates the angle between the square's normal and the Turtlebot's forward axis
// For accuracy, two normals are found and the average is taken between them
void LaserProcessing::normalAngleToBot(){
    //points for normal1
    CoordPoint3D p1, p2, normalVec1;
    p1.x = topLeft3DGlobal_.x - topRight3DGlobal_.x;
    p1.y = topLeft3DGlobal_.y - topRight3DGlobal_.y;
    p1.z = topLeft3DGlobal_.z - topRight3DGlobal_.z;
    p2.x = bottomRight3DGlobal_.x - topRight3DGlobal_.x;
    p2.y = bottomRight3DGlobal_.y - topRight3DGlobal_.y;
    p2.z = bottomRight3DGlobal_.z - topRight3DGlobal_.z;
    //calculate normal1
    normalVec1.x = p1.y * p2.z - p1.z * p2.y;
    normalVec1.y = p1.z * p2.x - p1.x * p2.z;
    normalVec1.z = p1.x * p2.y - p1.y * p2.x;
    //normalise into a unit vector
    double length = sqrt(pow(normalVec1.x,2) + pow(normalVec1.y,2) + pow(normalVec1.z,2));
        if (length != 0.0) {
            normalVec1.x /= length;
            normalVec1.y /= length;
            normalVec1.z /= length;
        }

    //points for normal2
    CoordPoint3D p3, normalVec2;
    p3.x = bottomLeft3DGlobal_.x - topRight3DGlobal_.x;
    p3.y = bottomLeft3DGlobal_.y - topRight3DGlobal_.y;
    p3.z = bottomLeft3DGlobal_.z - topRight3DGlobal_.z;
    //calculate normal2
    normalVec2.x = p3.y * p2.z - p3.z * p2.y;
    normalVec2.y = p3.z * p2.x - p3.x * p2.z;
    normalVec2.z = p3.x * p2.y - p3.y * p2.x;
    //normalise into a unit vector
    double length2 = sqrt(pow(normalVec2.x,2) + pow(normalVec2.y,2) + pow(normalVec2.z,2));
        if (length != 0.0) {
            normalVec2.x /= length2;
            normalVec2.y /= length2;
            normalVec2.z /= length2;
        }

    // Calculate average between the two vectors found
    CoordPoint3D normalVecAverage;
    normalVecAverage.x = (normalVec1.x + normalVec2.x) /2;
    normalVecAverage.y = (normalVec1.y + normalVec2.y) /2;
    normalVecAverage.z = (normalVec1.z + normalVec2.z) /2;

    // Print to terminal (for debugging)
    std::cout << "normalVec1: " << normalVec1.x <<", " << normalVec1.y << ", " << normalVec1.z << std::endl;
    std::cout << "normalVec2: " << normalVec2.x <<", " << normalVec2.y << ", " << normalVec2.z << std::endl;
    std::cout << "normalVecAverage: " << normalVecAverage.x <<", " << normalVecAverage.y << ", " << normalVecAverage.z << std::endl;

    // Intialise camera vector (forwards is local x-axis)
    CoordPoint3D cameraNormal;
    cameraNormal.x = 1;
    cameraNormal.y = 0;
    cameraNormal.z = 0;

    // Calculate the angle between the two vectors (square normal vector and camera vector)
    double dot = normalVecAverage.x * cameraNormal.x + normalVecAverage.y * cameraNormal.y + normalVecAverage.z * cameraNormal.z;
    double length_v1 = pow((normalVecAverage.x * normalVecAverage.x + normalVecAverage.y * normalVecAverage.y + normalVecAverage.z * normalVecAverage.z), 0.5);
    double length_v2 = pow((cameraNormal.x * cameraNormal.x + cameraNormal.y * cameraNormal.y + cameraNormal.z * cameraNormal.z), 0.5);
    double cos_theta = dot / (length_v1 * length_v2);

    // Calculate the angle in radians
    double thetaRad = acos(cos_theta);
    double thetaDeg = thetaRad*180/M_PI;
    //Print to terminal (for debugging)
    std::cout << "angle from normal: " << thetaRad << "rad" << std::endl;
}

// The following function converts the quaternion returned from the odometry orientation and returns the calculated yaw value in radians
double LaserProcessing::getYaw(geometry_msgs::Quaternion q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

// The following function finds the equation of the normal extending from the centre of the square
void LaserProcessing::findNormal() {
    double m = (bottomLeft3D_.y-bottomRight3D_.y)/(bottomLeft3D_.x-bottomRight3D_.x);
    double mn = -1/m;
    ROS_INFO_STREAM(mn);
    y_int_ = centreGlobal_.y - mn * centreGlobal_.x;
    angleGradient_ = atan2(mn,1);
    angleParallel_ = atan2(m,1);
    distance_ = y_int_/sqrt(mn*mn+1);
}

// The following function rotates the local coordinates about the global z-axis by a specific yaw value
LaserProcessing::CoordPoint3D LaserProcessing::rotateByYaw(LaserProcessing::CoordPoint3D localPoint, double yaw) {
    LaserProcessing::CoordPoint3D rotatedPoint;
    rotatedPoint.x = cos(yaw) * localPoint.x - sin(yaw) * localPoint.y;
    rotatedPoint.y = sin(yaw) * localPoint.x + cos(yaw) * localPoint.y;
    return rotatedPoint;
}

// The following function calculates the coordinates of a point relative to the Turtlebot in the global coordinate frame
LaserProcessing::CoordPoint3D LaserProcessing::localToGlobal(LaserProcessing::CoordPoint3D localPoint) {
    double yaw = getYaw(odo_.pose.pose.orientation);
    LaserProcessing::CoordPoint3D globalPoint = rotateByYaw(localPoint, yaw);
    globalPoint.x += odo_.pose.pose.position.x;
    globalPoint.y += odo_.pose.pose.position.y;
    globalPoint.z += 0;
    return globalPoint;
}