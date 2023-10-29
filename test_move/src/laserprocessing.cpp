#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <fstream> // header for saving images


using namespace std;
using namespace cv;

LaserProcessing::LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth, sensor_msgs::CameraInfo caminfo, nav_msgs::Odometry odo):
    image_(image), depth_(depth), squareDetected_(false), camInfo_(caminfo), odo_(odo)
{
}

void LaserProcessing::setSquareDetected(bool squareDetected) {
    squareDetected_ = squareDetected;
}

bool LaserProcessing::testMessages(double& y_int, double& turnAngle, double& distance) {
    ROS_INFO_STREAM("image test");
    ROS_INFO_STREAM(image_.header.seq);
    ROS_INFO_STREAM("depth test");
    ROS_INFO_STREAM(depth_.header.seq);
    convertToGreyscale();
    if(squareDetected_) {
        ROS_INFO_STREAM("base_cmd_turn_left");
        ROS_INFO_STREAM(odo_.pose.pose.position.x);
        ROS_INFO_STREAM(odo_.pose.pose.position.y);
        ROS_INFO_STREAM(odo_.pose.pose.position.z);
        ROS_INFO_STREAM(odo_.pose.pose.orientation.x);
        ROS_INFO_STREAM(odo_.pose.pose.orientation.y);
        ROS_INFO_STREAM(odo_.pose.pose.orientation.z);
        ROS_INFO_STREAM(odo_.pose.pose.orientation.w);
        
        normalAngleToBot();
        findNormal();
        // turnAngle = angleGradient_ - getYaw(odo_.pose.pose.orientation);
        turnAngle = angleParallel_;

        y_int = y_int_;
        distance = distance_;

        
    }

    return squareDetected_;
    
}

void LaserProcessing::findCentre(double& depth1, double& turnAngle1) {
    convertToGreyscale();
    double depth = (topLeft3D_.x + topRight3D_.x)/2;
    std::cout << "depth: " << depth << std::endl;
    double turnAngle = 0;
    if(squareDetected_) {
        double centreY = (topLeft3D_.y + topRight3D_.y)/2;
        std::cout << "centre: " << centreY << std::endl;
        if(centreY>0.05) {
            turnAngle = -atan(abs(centreY)/depth);
        }
        else if(centreY<-0.05) {
            turnAngle = atan(abs(centreY)/depth);
        }
    }
    depth1 = depth;
    turnAngle1 = turnAngle;
    std::cout << "turn angle: " << turnAngle << std::endl;
}

void LaserProcessing::convertToGreyscale() {
    std::string imageEncoding = image_.encoding;

    // Check if the image is already grayscale (mono8)
    if (imageEncoding == "mono8") {
        ROS_INFO("Image is already grayscale (mono8). No conversion needed.");
        return; // No need to convert
    }
    else if (imageEncoding == "rgb8"){
        ROS_INFO("MUST CONVERT FROM RGB8");
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
                uint8_t greyscale_value = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b); //CHECK LAB FOR CORRECT VALUES
                
                // Set the grayscale value in the image data
                greyscale_image_[yPixel * width + xPixel] = greyscale_value;
                
            }
        }
        src_gray = Mat(height, width, CV_8UC1, greyscale_image_.data());
        namedWindow( source_window, WINDOW_AUTOSIZE );
        createTrackbar("Threshold:", source_window, &thresh, max_thresh, staticCornerHarris, this);
        imshow( source_window, src_gray );
        cornerHarris_demo( 0, 0 );
        // Set the flag to indicate that the grayscale image is initialised
        greyscale_image_initialised_ = true;
        if (greyscale_image_initialised_){
            saveGreyscaleAsPGM("grayscale_image.pgm", greyscale_image_, width, height);
            ROS_INFO("CONVERTED");
        }
        return;
    }
    else{
    std::cout << "Image Encoding: " << imageEncoding << std::endl;
    }
}



// ****************************************************
// *************** CHATGPT **********************
// ****************************************************


void LaserProcessing::saveGreyscaleAsPGM(const std::string& filename, const std::vector<uint8_t>& grayscale_image, int width, int height)
{
    std::ofstream outfile(filename.c_str(), std::ios::binary);
    if (!outfile)
    {
        std::cerr << "Error: Unable to open the output file." << std::endl;
        return;
    }

    // Write PGM header
    outfile << "P5\n";                     // P5 indicates binary PGM
    outfile << width << " " << height << "\n"; // Image dimensions
    outfile << "255\n";                   // Maximum pixel value (8-bit grayscale)

    // Write image data
    outfile.write(reinterpret_cast<const char*>(grayscale_image.data()), grayscale_image.size());

    outfile.close();

    std::cout << "Grayscale image saved as " << filename << std::endl;
}


// ****************************************************
// *************** OPENCV **********************
// ****************************************************
void LaserProcessing::cornerHarris_demo( int, void* )
{
    setSquareDetected(false);
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( src_gray.size(), CV_32FC1 );
    int blockSize = 3;
    int apertureSize = 3;
    double k = 0.04;
    cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    corner_x_coords.clear();
    corner_y_coords.clear();
    if( (int) dst_norm.at<float>(1,1) < thresh) {
    for( int j = 0; j < dst_norm.rows ; j++ )
    {
        for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                // Print the corner location (coordinates)
                std::cout << "Corner detected at (" << i << ", " << j << ")" << std::endl;
                corner_x_coords.push_back(i);
                corner_y_coords.push_back(j);

                circle( dst_norm_scaled, Point( i, j ), 5, Scalar(0), 2, 8, 0 );
            }
        }
    }
    }
    if(corner_x_coords.size()>3 && corner_x_coords.size()<25) {
        setSquareDetected(true);
        x1_ = corner_x_coords[0];
        y1_ = corner_y_coords[0];
        // for (int i = 1; i < corner_x_coords.size(); i++) {
        //     if(corner_x_coords[i]-x1_ > 10) x2_ = corner_x_coords[i];
        // }
        // for (int i = 1; i < corner_y_coords.size(); i++) {
        //     if(corner_y_coords[i]-y1_ > 10) y2_ = corner_y_coords[i];
        // }

        squareCornersX_.clear();
        squareCornersY_.clear();

        squareCornersX_.push_back(corner_x_coords.at(0));
        squareCornersY_.push_back(corner_y_coords.at(0));

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
            if(squareCornersX_.size() == 4) break;
        }

        
        
        if(squareCornersX_.size() < 4) setSquareDetected(false);
        else {
            std::cout << "x1,y1: " << squareCornersX_.at(0) << ", " << squareCornersY_.at(0) << std::endl;
            std::cout << "x2,y2: " << squareCornersX_.at(1) << ", " << squareCornersY_.at(1) << std::endl;
            std::cout << "x3,y3: " << squareCornersX_.at(2) << ", " << squareCornersY_.at(2) << std::endl;
            std::cout << "x4,y4: " << squareCornersX_.at(3) << ", " << squareCornersY_.at(3) << std::endl;

            int sumX = 0;
            int sumY = 0;
            for(int i=0; i<4; i++) {
                sumX += squareCornersX_.at(i);
                sumY += squareCornersY_.at(i);
            }

            centreX_ = sumX/4;
            centreY_ = sumY/4;
            
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

            std::cout << "top left: " << topLeft_.x << ", " << topLeft_.y << std::endl;
            std::cout << "top right: " << topRight_.x << ", " << topRight_.y << std::endl;
            std::cout << "bottom left: " << bottomLeft_.x << ", " << bottomLeft_.y << std::endl;
            std::cout << "bottom right: " << bottomRight_.x << ", " << bottomRight_.y << std::endl;

            calculate3DCoords();
            // calculate3DNormal();
        }
    }
    
    // namedWindow( corners_window, WINDOW_AUTOSIZE );
    // imshow( corners_window, dst_norm_scaled );
}


void LaserProcessing::staticCornerHarris(int value, void* userdata) {
        LaserProcessing* instance = reinterpret_cast<LaserProcessing*>(userdata);
        instance->cornerHarris_demo(value, userdata);
    }

void LaserProcessing::calculate3DCoords() {
    //principle points
    double cx = 960.5;
    double cy = 540.5;
    //focal lengths
    double fx = 1206.8897719532354;
    double fy = 1206.8897719532354;
    //square size (real world?)
    double d = 0.5;

    //topLeft3D
        topLeft3D_.y = (topLeft_.x-cx)*((d*fy)/(bottomLeft_.y-topLeft_.y))*(1/fx);
        topLeft3D_.z = (topLeft_.y-cy)*((d*fy)/(topLeft_.y-bottomLeft_.y))*(1/fy);
        topLeft3D_.x = ((d*fy)/(bottomLeft_.y-topLeft_.y));
    //topRight3D
        topRight3D_.y = (topRight_.x-cx)*((d*fy)/(bottomRight_.y-topRight_.y))*(1/fx);
        topRight3D_.z = (topRight_.y-cy)*((d*fy)/(topRight_.y-bottomRight_.y))*(1/fy);
        topRight3D_.x = ((d*fy)/(bottomRight_.y-topRight_.y));
    //bottomLeft3D
        bottomLeft3D_.y = (bottomLeft_.x-cx)*((d*fy)/(bottomLeft_.y-topLeft_.y))*(1/fx);
        bottomLeft3D_.z = (bottomLeft_.y-cy)*((d*fy)/(topLeft_.y-bottomLeft_.y))*(1/fy);
        bottomLeft3D_.x = ((d*fy)/(bottomLeft_.y-topLeft_.y));
    //bottomRight3D
        bottomRight3D_.y = (bottomRight_.x-cx)*((d*fy)/(bottomRight_.y-topRight_.y))*(1/fx);
        bottomRight3D_.z = (bottomRight_.y-cy)*((d*fy)/(topRight_.y-bottomRight_.y))*(1/fy);
        bottomRight3D_.x = ((d*fy)/(bottomRight_.y-topRight_.y));

    std::cout << "topLeft3D: " << topLeft3D_.x <<", " << topLeft3D_.y << ", " << topLeft3D_.z << std::endl;
    std::cout << "topRight3D: " << topRight3D_.x <<", " << topRight3D_.y << ", " << topRight3D_.z << std::endl;
    std::cout << "bottomLeft3D: " << bottomLeft3D_.x <<", " << bottomLeft3D_.y << ", " << bottomLeft3D_.z << std::endl;
    std::cout << "bottomRight3D: " << bottomRight3D_.x <<", " << bottomRight3D_.y << ", " << bottomRight3D_.z << std::endl;

    topLeft3DGlobal_ = LaserProcessing::localToGlobal(topLeft3D_);
    topRight3DGlobal_ = LaserProcessing::localToGlobal(topRight3D_);
    bottomLeft3DGlobal_ = LaserProcessing::localToGlobal(bottomLeft3D_);
    bottomRight3DGlobal_ = LaserProcessing::localToGlobal(bottomRight3D_);

    std::cout << "topLeft3D global: " << topLeft3DGlobal_.x <<", " << topLeft3DGlobal_.y << ", " << topLeft3DGlobal_.z << std::endl;
    std::cout << "topRight3D global: " << topRight3DGlobal_.x <<", " << topRight3DGlobal_.y << ", " << topRight3DGlobal_.z << std::endl;
    std::cout << "bottomLeft3D global: " << bottomLeft3DGlobal_.x <<", " << bottomLeft3DGlobal_.y << ", " << bottomLeft3DGlobal_.z << std::endl;
    std::cout << "bottomRight3D global: " << bottomRight3DGlobal_.x <<", " << bottomRight3DGlobal_.y << ", " << bottomRight3DGlobal_.z << std::endl;

    centreGlobal_.x = (bottomRight3DGlobal_.x + bottomLeft3DGlobal_.x)/2;
    centreGlobal_.y = (bottomRight3DGlobal_.y + bottomLeft3DGlobal_.y)/2;
    centreGlobal_.z = 0;

}

// void LaserProcessing::calculate3DNormal() {
//     CoordPoint3D normalVec1 = crossProduct(topLeft3D_ - topRight3D_, bottomRight3D_ - topRight3D_);
//     CoordPoint3D normalVec2 = crossProduct(bottomRight3D_ - topRight3D_, bottomLeft3D_ - topRight3D_);
//     CoordPoint3D estimatedNormal = normalize(normalVec1) + normalize(normalVec2);
//     estimatedNormal = normalize(estimatedNormal);
    
//     std::cout << "estimated normal: " << estimatedNormal.x << ", " << estimatedNormal.y << ", " << estimatedNormal.z << std::endl;

// }
// CoordPoint3D LaserProcessing::crossProduct(const CoordPoint3D& p1, const CoordPoint3D& p2){
//     double x = p1.y * p2.z - p1.z * p2.y;
//     double y = p1.z * p2.x - p1.x * p2.z;
//     double z = p1.x * p2.y - p1.y * p2.x;
//     CoordPoint3D crossProd;
//     crossProd.x = x;
//     crossProd.y = y;
//     crossProd.z = z;
//     return crossProd;
// }

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
    CoordPoint3D p3, p4, normalVec2;
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

    CoordPoint3D normalVecAverage;
    normalVecAverage.x = (normalVec1.x + normalVec2.x) /2;
    normalVecAverage.y = (normalVec1.y + normalVec2.y) /2;
    normalVecAverage.z = (normalVec1.z + normalVec2.z) /2;

    std::cout << "normalVec1: " << normalVec1.x <<", " << normalVec1.y << ", " << normalVec1.z << std::endl;
    std::cout << "normalVec2: " << normalVec2.x <<", " << normalVec2.y << ", " << normalVec2.z << std::endl;
    std::cout << "normalVecAverage: " << normalVecAverage.x <<", " << normalVecAverage.y << ", " << normalVecAverage.z << std::endl;

    CoordPoint3D cameraNormal;
    cameraNormal.x = 1;
    cameraNormal.y = 0;
    cameraNormal.z = 0;

    double dot = normalVecAverage.x * cameraNormal.x + normalVecAverage.y * cameraNormal.y + normalVecAverage.z * cameraNormal.z;
    double length_v1 = pow((normalVecAverage.x * normalVecAverage.x + normalVecAverage.y * normalVecAverage.y + normalVecAverage.z * normalVecAverage.z), 0.5);
    double length_v2 = pow((cameraNormal.x * cameraNormal.x + cameraNormal.y * cameraNormal.y + cameraNormal.z * cameraNormal.z), 0.5);
    double cos_theta = dot / (length_v1 * length_v2);

    // Calculate the angle in radians
    double thetaRad = acos(cos_theta);
    double thetaDeg = thetaRad*180/M_PI;

    // normalVecAverage.y = 0;
    // double angle1 = atan2(normalVecAverage.x, normalVecAverage.z);
    // double angle2 = atan2(cameraNormal.x, cameraNormal.z);
    // double thetaRad = angle2 - angle1;
    // double thetaDeg = thetaRad*180/M_PI;

    std::cout << "theta: " << thetaRad << "rad" << std::endl;
    std::cout << "theta: " << thetaDeg << "deg" << std::endl;

}

double LaserProcessing::getYaw(geometry_msgs::Quaternion q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
     std::cout << "yaw: " << yaw << "rad" << std::endl;

    return yaw;
}

LaserProcessing::CoordPoint3D LaserProcessing::local2Global(LaserProcessing::CoordPoint3D corner, nav_msgs::Odometry odo) {   
    double yaw = getYaw(odo.pose.pose.orientation);
    double newX = odo.pose.pose.position.x + corner.x*cos(yaw) - corner.y*sin(yaw);
    double newY = odo.pose.pose.position.y + corner.x*cos(yaw);

    LaserProcessing::CoordPoint3D p;
    p.x = newX;
    p.y = newY;
    p.z = corner.z;
    return p;
}

void LaserProcessing::findNormal() {
    double m = (bottomLeft3D_.y-bottomRight3D_.y)/(bottomLeft3D_.x-bottomRight3D_.x);
    double mn = -1/m;
    ROS_INFO_STREAM(mn);
    y_int_ = centreGlobal_.y - mn * centreGlobal_.x;
    angleGradient_ = atan2(mn,1);
    angleParallel_ = atan2(m,1);
    distance_ = y_int_/sqrt(mn*mn+1);
}

LaserProcessing::CoordPoint3D LaserProcessing::rotateByYaw(LaserProcessing::CoordPoint3D localPoint, double yaw) {
    LaserProcessing::CoordPoint3D rotatedPoint;
    rotatedPoint.x = cos(yaw) * localPoint.x - sin(yaw) * localPoint.y;
    rotatedPoint.y = sin(yaw) * localPoint.x + cos(yaw) * localPoint.y;
    return rotatedPoint;
}

LaserProcessing::CoordPoint3D LaserProcessing::localToGlobal(LaserProcessing::CoordPoint3D localPoint) {
    double yaw = getYaw(odo_.pose.pose.orientation);
    LaserProcessing::CoordPoint3D globalPoint = rotateByYaw(localPoint, yaw);
    globalPoint.x += odo_.pose.pose.position.x;
    globalPoint.y += odo_.pose.pose.position.y;
    globalPoint.z += 0;
    return globalPoint;
}