#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <fstream> // header for saving images


using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::Image image, sensor_msgs::PointCloud2 depth):
    image_(image), depth_(depth)
{
}

void LaserProcessing::testMessages() {
    ROS_INFO_STREAM("image test");
    ROS_INFO_STREAM(image_.header.seq);
    ROS_INFO_STREAM("depth test");
    ROS_INFO_STREAM(depth_.header.seq);
    convertToGreyscale();
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



// ********************************************************
// ******************* CHATGPT **************************
// ********************************************************


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