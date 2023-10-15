#pragma once
#include <boilerplate.hpp>
#include <VisionStage.hpp>


struct CopyConfig
{
    unsigned int input_channels=4;
    int skip_input_channel=-1;
    unsigned int subsample_major=1, subsample_minor=1;
    int rect[4] = {0, 0, -1, -1}; // xmin, ymin, x_max, y_max

    // Return net scale factor for the above subsample steps
    float get_scale_factor();
};


class ImageSource: public VisionStage
{
    protected:
    unsigned int image_width, image_height;
    Mat raw_image;
    string color_space;
    bool flip_vertical = false;
    bool flip_horizontal = false;
    unsigned int frame_count=0;
    unsigned int channels = 3;

    bool publish = false;
    unsigned int publish_frame_interval = 1;
    unsigned int publish_subsample_major = 1, publish_subsample_minor = 1;
    rclcpp::Publisher<msg_ImageSource>::SharedPtr publisher_;
    bool publishing_in_progress = false;
    
    public:
    ImageSource(string&, Json::Value);
    void configure(Json::Value);
    void reconfigure(Json::Value config);
    virtual void acquire_image(unordered_map<string, shared_ptr<VisionStage>>&) = 0;
    void execute(unordered_map<string, shared_ptr<VisionStage>> &);
    void publish_image(long int);
    // Copy image to a destination buffer, possibly with subsampling and skipping an input channel.
    template<typename T> void copy_to_buffer(T *, struct CopyConfig&);
    // Copy image to OpenCV Mat and return it
    Mat copy_to_mat(struct CopyConfig&);
    virtual Mat& get_image_matrix() = 0;
    Vec2i get_image_size();
    // Return a copy config for the given roi in normalized coordinates
    void update_copy_config_roi(struct CopyConfig&, Vec4f);
    void update_copy_config_roi(struct CopyConfig&, Vec2f, Vec2i);
};


// TODO: Make this function static
template<typename T> void ImageSource::copy_to_buffer(T * buf, struct CopyConfig &copy_cfg)
{
    auto image_mat = get_image_matrix();
    unsigned char* imgraw = image_mat.ptr<unsigned char>(0);
    auto imgraw_orig = imgraw;

    int i = 0;
    bool sr=true, sc=true;
    int pr = 0;

    unsigned int subsample_major = copy_cfg.subsample_major;
    unsigned int subsample_minor = copy_cfg.subsample_minor;

    unsigned int channels = copy_cfg.input_channels;

    unsigned int rmin = copy_cfg.rect[1], cmin = copy_cfg.rect[0];
    unsigned int rmax = (copy_cfg.rect[3]==-1)?image_height-1:copy_cfg.rect[3];
    unsigned int cmax = (copy_cfg.rect[2]==-1)?image_width-1:copy_cfg.rect[2];

    if (color_space=="YUYV")
    {
        cmin -= cmin%2;
        cmax -= (1-cmax%2);
    }

    // Define these things so that we don't have to compute them during the loop
    int bytes_per_pixel = ((color_space=="YUYV")?2:channels);
    int pixels_per_channel_group = ((color_space=="YUYV")?2:1);

    unsigned int r_pad = bytes_per_pixel * ((image_width - (cmax+1)) + cmin);

    int shift_r_major = r_pad + (bytes_per_pixel*image_width)*(subsample_major-1);
    int shift_r_minor = r_pad + (bytes_per_pixel*image_width)*(subsample_minor-1);

    int shift_c_major = channels*(subsample_major-1);
    int shift_c_minor = channels*(subsample_minor-1);

    int inc_c_major = pixels_per_channel_group*subsample_major;
    int inc_c_minor = pixels_per_channel_group*subsample_minor;

    int inc_r_major = subsample_major;
    int inc_r_minor = subsample_minor;

    // Offset starting position
    imgraw += bytes_per_pixel * pixels_per_channel_group * ((cmin+ image_width*rmin)/pixels_per_channel_group);

    for(int r = rmin; r <= rmax;) {
        sc = true;
        int c = cmin;
        for(; c <= cmax;) {
            for(int b=0; b<channels; ++b, ++imgraw)
                if(b != copy_cfg.skip_input_channel)
                    *(buf++) = ((*(imgraw)));
            if(sc)
            {
                c += inc_c_major;
                imgraw += shift_c_major;
            }
            else
            {
                c += inc_c_minor;
                imgraw += shift_c_minor;
            }
            sc = (!sc);
        }
        int c_overshoot = (cmax+1-c)*bytes_per_pixel;
        imgraw += c_overshoot;
        if(sr)
        {
            r += inc_r_major;
            imgraw += shift_r_major;
        }
        else
        {
            r += inc_r_minor;
            imgraw += shift_r_minor;
        }
        sr = (!sr);
    }
}
