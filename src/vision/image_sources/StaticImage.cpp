#include <StaticImage.hpp>

#define YUYV_BYTES_TO_DROP 36

std::vector<char> readFileBytes(const char* filename)
{
    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data:
    std::vector<char> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}

StaticImage::StaticImage(string& name_, Json::Value& config) : ImageSource(name_, config)
{
    configure(config);
}


void StaticImage::configure(Json::Value config)
{
    ImageSource::configure(config);
    // Read details from config
    string image_filename = config["image_filename"].asString();

    // Read image from file and store in specified color format

    if(color_space == "YUYV")
    {
        auto yuyv_bytes = readFileBytes(image_filename.c_str());
        Mat(image_height, image_width, CV_8UC2, yuyv_bytes.data()+YUYV_BYTES_TO_DROP).copyTo(raw_image);
    }
    else
        raw_image = imread(image_filename);

    RCLCPP_DEBUG(this->get_logger(), "Static Image: %d channels, width %d height %d.", raw_image.channels(), raw_image.size().width, raw_image.size().height);
}


void StaticImage::acquire_image(unordered_map<string, shared_ptr<VisionStage>>& stages)
{
    RCLCPP_DEBUG(this->get_logger(), "Static Image. Nothing to do.");
}

Mat& StaticImage::get_image_matrix()
{
    return raw_image;
}
