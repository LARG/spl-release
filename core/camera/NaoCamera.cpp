/**
 * \file Platform/linux/NaoCamera.cpp
 * Interface to the Nao camera using linux-uvc.
 * \author Colin Graf
 * \author Thomas R�fer
 * \author Todd Hester
 */

#include "NaoCamera.h"
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <chrono>

// TODO: for camera color issue can try:
// 1. disable/enable streaming
// 2. init camera defaults (init/width)
// X. init image format (yuv, size)
// 4. set initial settings
// X. try req/mapping buffers for each camera?
// X. try init camera settings after mem map?
// 7. if these don't work...
//    a. maybe disable streaming while changing them?
//    b. maybe unmap buffers and re-map after change?
//    c. possibly after camera default init.. we have re-init the other things (framerate, format, etc)?

NaoCamera::NaoCamera(const ImageParams& iparams, CameraParams& camera_params, CameraParams& read_camera_params) : iparams_(iparams), nextBuf(0), prevBuf(0), timeStamp(0), storedTimeStamp(0), initialized(false), camera_params_(camera_params), read_camera_params_(read_camera_params) {}

void NaoCamera::init() {

  if(!initialized) {
    initOpenVideoDevice();
  }
  initSetImageFormat();
  initSetFrameRate();

  if (!initialized) {
    initRequestAndMapBuffers();
    initQueueAllBuffers();
  }

  enableStreaming();
  setDefaultSettings();

  initialized = true;
}

void NaoCamera::reset() {
  disableStreaming();
  init();
}

bool NaoCamera::selfTest(){
  uint8_t* image = getImage();
  for(int i=0; i <= iparams_.rawSize - 2; i+=2){
    int y = image[i];
    int uv = image[i + 1];
    if(uv < 120 || uv > 140)
      return true; // non-black area of image found, test passed
  }
  return false; // entire image is black
}

// Turns on automatic white balancing
void NaoCamera::enableAutoWB(){

  if(!setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 1)) std::cerr << "Error setting auto white balance: " << errno << "\n";

}

// Reads the current white balance and locks it to that by turning off auto WB
int NaoCamera::lockWB(){

  int wb = getControlSetting(V4L2_CID_DO_WHITE_BALANCE);
  if(!setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 0)) std::cerr << "Error setting manual white balance: " << errno << "\n";
  if(!setControlSetting(V4L2_CID_DO_WHITE_BALANCE, wb)) std::cerr << "Error setting manual white balance: " << errno << "\n"; // 2700-6500
  return wb;
}

NaoCamera::~NaoCamera() {
  disableStreaming();
  unmapAndFreeBuffers();
  close(videoDeviceFd);
}

void NaoCamera::unmapAndFreeBuffers() {
  for(int i = 0; i < frameBufferCount; ++i)
    munmap(mem[i], memLength[i]);
  free(buf);
  free(prevBuf);
  free(nextBuf);
}

void NaoCamera::initRequestAndMapBuffers() {
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  int result = ioctl(videoDeviceFd, VIDIOC_REQBUFS, &rb);
  if(result < 0) std::cout << "NaoCamera: Error requesting buffers\n";

  // map the buffers
  buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
  prevBuf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
  nextBuf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
  for(int i = 0; i < frameBufferCount; ++i) {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    result = ioctl(videoDeviceFd, VIDIOC_QUERYBUF, buf);
    if(result < 0) std::cout << "NaoCamera: Error querying buffers\n";
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, videoDeviceFd, buf->m.offset);
    if(mem[i] == MAP_FAILED) std::cout << "NaoCamera: Error mmapping video device\n";
  }
}

void NaoCamera::initQueueAllBuffers() {
  for(int i = 0; i < frameBufferCount; ++i) {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    int result = ioctl(videoDeviceFd, VIDIOC_QBUF, buf);
    if(result < 0) std::cout << "NaoCamera: Error queueing buffers\n";
  }
}

void NaoCamera::enableStreaming() {
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int result = ioctl(videoDeviceFd, VIDIOC_STREAMON, &type);
  if(result < 0) std::cout << "NaoCamera: Error enabling streaming\n";
}

void NaoCamera::disableStreaming() {
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int result = ioctl(videoDeviceFd, VIDIOC_STREAMOFF, &type);
  if(result < 0) std::cout << "NaoCamera: Error disabling streaming\n";
}

void NaoCamera::initOpenVideoDevice() {
  std::cout << "NaoCamera: Opening device at '" << device_path_ << "'" << std::endl;

  struct stat st;
  if (-1 == stat(device_path_.c_str(), &st)) {
    std::cerr << "NaoCamera(Fatal): Cannot identify '" << device_path_
      << "': " << errno << ", " << strerror(errno) << std::endl;
    exit (EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    std::cerr << "NaoCamera(Fatal): '" << device_path_
      << "' does not exist! " << std::endl;
    exit (EXIT_FAILURE);
  }

  videoDeviceFd = open (device_path_.c_str(), O_RDWR, 0);

  if (-1 == videoDeviceFd) {
    std::cerr << "NaoCamera(Fatal): Cannot open '" << device_path_
      << "': " << errno << ", " << strerror(errno) << std::endl;
    exit (EXIT_FAILURE);
  }
}

void NaoCamera::initSetImageFormat() {

  std::cout << "NaoCamera: Setting camera image format" << std::endl;

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = iparams_.width;
  fmt.fmt.pix.height = iparams_.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // native format of camera
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  printf("image size: width: %i, height: %i\n", fmt.fmt.pix.width, fmt.fmt.pix.height);

  bool retval = ioctl(videoDeviceFd, VIDIOC_S_FMT, &fmt);
  printf("received bytes per line: %i\n", fmt.fmt.pix.bytesperline);
  if (retval == -1) {
    std::cerr << "NaoCamera(Fatal): Error initializing camera format" << std::endl;
    exit (EXIT_FAILURE);
  }

  if (fmt.fmt.pix.bytesperline != iparams_.width * 2 || fmt.fmt.pix.sizeimage != iparams_.rawSize){
    std::cerr << "NaoCamera(Fatal): Format size unexpected. Incompatible!" << std::endl;
    exit (EXIT_FAILURE);
  }
}

void NaoCamera::initSetFrameRate() {

  std::cout << "NaoCamera: Setting camera frame rate" << std::endl;

  // set frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(struct v4l2_streamparm));

  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int result = ioctl(videoDeviceFd, VIDIOC_G_PARM, &fps);
  if(result == -1) std::cout << "NaoCamera: Error getting video capture params\n";

  fps.parm.capture.timeperframe.numerator = 1;
  fps.parm.capture.timeperframe.denominator = 30;
  result = ioctl(videoDeviceFd, VIDIOC_S_PARM, &fps);

  if(result == -1) std::cout << "NaoCamera: Error setting frame rate\n";
}


int NaoCamera::set_uvc_xu(int device_fd, uint8_t extension_unit_id, uint8_t control_selector,
    uint16_t size, uint8_t* data)
{


  std::this_thread::sleep_for(std::chrono::milliseconds(100));


  struct uvc_xu_control_query query;
  query.unit = extension_unit_id;
  query.selector = control_selector;
  query.query = UVC_SET_CUR;
  query.size = size;
  query.data = data;
  return ioctl(device_fd, UVCIOC_CTRL_QUERY, &query);
}


int NaoCamera::get_uvc_xu(int device_fd, uint8_t control_selector, uint16_t size, uint8_t* data)
{

  // Extension unit is always 3
  uint8_t extension_unit_id = 3;

  struct uvc_xu_control_query query;
  query.unit = extension_unit_id;
  query.selector = control_selector;
  query.query = UVC_GET_CUR;
  query.size = size;
  query.data = data;
  return ioctl(device_fd, UVCIOC_CTRL_QUERY, &query);
}


void NaoCamera::setDefaultSettings() {
  std::cout << "NaoCamera: Initializing Camera Parameters to default" << std::endl;
  //  if(!setControlSetting(V4L2_CID_HFLIP, hflip_)) std::cerr << "Error setting hflip: " << errno << "\n";
  //  if(!setControlSetting(V4L2_CID_VFLIP, vflip_)) std::cerr << "Error setting vflip: " << errno << "\n";



  // NEW AUTOEXPOSURE SETTINGS!!!!!!
  //////////////////////////////////////////


  uint8_t aec[] = {0x1, 0x35, 0x03, 0x00, 0x00};
  set_uvc_xu(videoDeviceFd, 3, 14, 5, aec);

  // Top threshold
  uint8_t top1[] = {0xFF, 0x3A, 0x0F, 0x00, 0x20};
  uint8_t top2[] = {0xFF, 0x3A, 0x1B, 0x00, 0x20};
  set_uvc_xu(videoDeviceFd, 3, 14, 5, top1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, top2);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, top1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, top2);

  // 0x24 -- 0x1A   -- LEFT SIDE FIELD


  // Bottom threshold
  uint8_t bot1[] = {0xFF, 0x3A, 0x1E, 0x00, 0x18};
  uint8_t bot2[] = {0xFF, 0x3A, 0x10, 0x00, 0x18};
  set_uvc_xu(videoDeviceFd, 3, 14, 5, bot1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, bot2);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, bot1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, bot2);



  // Night mode must be enabled for autoexposure to work properly in dark settings
  uint8_t nightMode[] = {0xFF, 0x3A, 0x00, 0x00, 0x7C};
  set_uvc_xu(videoDeviceFd, 3, 14, 5, nightMode);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, nightMode);


  // 4x4 window. Clear the weights for the windows on the top half of the image, so that the autoexposure only uses the bottom half to calculate exposure. 
  uint8_t window0[] = {0xFF, 0x56, 0x88, 0x00, 0x00};
  uint8_t window1[] = {0xFF, 0x56, 0x89, 0x00, 0x00};
  uint8_t window2[] = {0xFF, 0x56, 0x8A, 0x00, 0x00};
  uint8_t window3[] = {0xFF, 0x56, 0x8B, 0x00, 0x00};

  set_uvc_xu(videoDeviceFd, 3, 14, 5, window0);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window2);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window3);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window0);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window1);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window2);
  set_uvc_xu(videoDeviceFd, 3, 14, 5, window3);



  //////////////////////////////////



  // Manual Exposure Settings  -- INDOOR FIELDS
  //////////////////////////////////////////////////
  /*  
      uint8_t aec[] = {0x1, 0x35, 0x03, 0x00, 0x01};
      set_uvc_xu(videoDeviceFd, 3, 14, 5, aec);

      uint8_t manualExp1[] = {0xFF, 0x35, 0x00, 0x00, 0x0F};
      uint8_t manualExp2[] = {0xFF, 0x35, 0x01, 0x00, 0xFF};
      uint8_t manualExp3[] = {0xFF, 0x35, 0x02, 0x00, 0xF0};

      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp1);
      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp2);
      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp3);
      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp1);
      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp2);
      set_uvc_xu(videoDeviceFd, 3, 14, 5, manualExp3);

      if(!setControlSetting(V4L2_CID_BRIGHTNESS, brightness)) std::cerr << "Error setting brightness: " << errno << "\n"; // 0-255, default: 55 -- Average brightness for autoexposure algorithms
   */
  ///////////////////////////////////////////////////




  /*
     set_uvc_xu(videoDeviceFd, 3, 14, 5, luminance);

     printf("=============================\n");
     printf("LUMINANCE: %X %X\n", luminance[3], luminance[4]);
     printf("=============================\n");
   */




  // Read the average target luminance
  uint8_t luminance[] = {0x00, 0x56, 0xA1, 0x00, 0x00};
  printf("Accessing registers: %X %X\n", luminance[1], luminance[2]);
  if (get_uvc_xu(videoDeviceFd, 14, 5, luminance)) std::cerr << "Error getting luminance: " << errno << "\n";
  printf("=============================\n");
  printf("LUMINANCE: %X %X\n", luminance[3], luminance[4]);
  printf("=============================\n");




  // Vertical flip
  uint8_t vflip = vflip_;
  set_uvc_xu(videoDeviceFd, 3, 13, 2, &vflip); 

  // Horizontal flip
  uint8_t hflip = hflip_;
  set_uvc_xu(videoDeviceFd, 3, 12, 2, &hflip);

  // This doesn't seem to do anything
  //  uint8_t luminance = 60000;
  //  set_uvc_xu(videoDeviceFd, 3, 8, 2, &luminance);



  // Turn off autofocus
  if(!setControlSetting(V4L2_CID_FOCUS_AUTO, 0)) std::cerr << "Error setting auto focus: " << errno << "\n";
  if(!setControlSetting(V4L2_CID_FOCUS_ABSOLUTE, 0)) std::cerr << "Error setting absolute focus: " << errno << "\n";



  /*



  ////// MANUAL WHITE BALANCE

  // White Balance
  //  if(!setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 0)) std::cerr << "Error setting auto white balance: " << errno << "\n";
  // Only valid for manual white balance. This sets the white balance temperature to a certain value. Note that technically it should have been implemented with V4L2_CID_WHITE_BALANCE_TEMPERATURE but Aldebaran screwed up on their drivers. 
  //  if(!setControlSetting(V4L2_CID_DO_WHITE_BALANCE, 2700)) std::cerr << "Error setting manual white balance: " << errno << "\n"; // 2700-6500


  /// AUTO WHITE BALANCE

  if(!setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 1)) std::cerr << "Error setting auto white balance: " << errno << "\n";

  /////// END

  //  std::cout << "\nWB: " << getControlSetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE) << std::endl;

   */
  // This is probably unnecessary now that we set it in the registers.
  //  if(!setControlSetting(V4L2_CID_EXPOSURE_AUTO, 0)) std::cerr << "Error setting exposure auto: " << errno << "\n";



  // This doesn't seem to make a difference
  //  if(!setControlSetting(V4L2_CID_BACKLIGHT_COMPENSATION, 0)) std::cerr << "Error backlight compensation brightness: " << errno << "\n";

  // Only valid for auto exposure 
  //  if(!setControlSetting(V4L2_CID_BRIGHTNESS, 0)) std::cerr << "Error setting brightness: " << errno << "\n"; // 0-255, default: 55 -- Average brightness for autoexposure algorithms

  /*

     if(!setControlSetting(V4L2_CID_CONTRAST, 48)) std::cerr << "Error setting contrast: " << errno << "\n"; //20-53 // 53
     if(!setControlSetting(V4L2_CID_SATURATION, 128)) std::cerr << "Error setting saturation: " << errno << "\n"; //0-255. 0 is like grayscale, 255 is tooo much color
     if(!setControlSetting(V4L2_CID_HUE, 0)) std::cerr << "Error setting hue: " << errno << "\n"; // ONLY VALUE
     if(!setControlSetting(V4L2_CID_SHARPNESS, 7)) std::cerr << "Error setting sharpness: " << errno << "\n"; //0-7
   */

  // These settings only apply for manual exposure
  // We have to set the camera exposure twice because of Aldebaran's world-class hardware.
  //  if(!setControlSetting(V4L2_CID_EXPOSURE, 300)) std::cerr << "Error setting exposure: " << errno << "\n";
  //  if(!setControlSetting(V4L2_CID_EXPOSURE, 300)) std::cerr << "Error setting exposure: " << errno << "\n";           // This only works if autoexposure is off (i.e. using manual exposure)
  //  if(!setControlSetting(V4L2_CID_GAIN, 64)) std::cerr << "Error setting gain: " << errno << "\n"; // 0-255         // 32





  assert(exposure <= 300); // "If you set exposure to higher than 300 the framerate will drop below 30hz, change the gain instead");
}

void NaoCamera::setCameraParams(){
  std::cout << "NaoCamera: Setting Camera Parameters (on the fly)" << std::endl;

  // if(!setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, camera_params_.kCameraAutoWhiteBalance)) std::cerr << "Error setting auto white balance: " << errno << "\n";
  // if(!setControlSetting(V4L2_CID_EXPOSURE_AUTO, camera_params_.kCameraExposureAuto)) std::cerr << "Error setting exposure auto: " << errno << "\n";
  // if(!setControlSetting(V4L2_CID_BACKLIGHT_COMPENSATION, camera_params_.kCameraBacklightCompensation)) std::cerr << "Error setting backlight compensation: " << errno << "\n";

  if(!setControlSetting(V4L2_CID_BRIGHTNESS, camera_params_.kCameraBrightness)) std::cerr << "Error setting brightness: " << errno << "\n"; //ONLY VALUE
  if(!setControlSetting(V4L2_CID_CONTRAST, camera_params_.kCameraContrast)) std::cerr << "Error setting contrast: " << errno << "\n"; //0-53
  if(!setControlSetting(V4L2_CID_SATURATION, camera_params_.kCameraSaturation)) std::cerr << "Error setting saturation: " << errno << "\n"; //0-255
  if(!setControlSetting(V4L2_CID_HUE, camera_params_.kCameraHue)) std::cerr << "Error setting hue: " << errno << "\n"; // ONLY VALUE
  //if(!setControlSetting(V4L2_CID_EXPOSURE, camera_params_.kCameraExposure)) std::cerr << "Error setting exposure: " << errno << "\n"; // ONLY VALUE
  //if(!setControlSetting(V4L2_CID_GAIN, camera_params_.kCameraGain)) std::cerr << "Error setting gain: " << errno << "\n"; // ONLY VALUE
  if(!setControlSetting(V4L2_CID_SHARPNESS, camera_params_.kCameraSharpness)) std::cerr << "Error setting sharpness: " << errno << "\n"; //0-4
}

void NaoCamera::enumerateSettingMenu() {
  printf ("  Menu items:\n");

  memset (&querymenu, 0, sizeof (querymenu));
  querymenu.id = queryctrl.id;

  for (querymenu.index = queryctrl.minimum;
      querymenu.index <= queryctrl.maximum;
      querymenu.index++) {
    if (0 == ioctl (videoDeviceFd, VIDIOC_QUERYMENU, &querymenu)) {
      printf ("  %s\n", querymenu.name);
    } else {
      perror ("  Setting is menu type, but menu could not be queried (VIDIOC_QUERYMENU)");
      printf ("  Displaying min,max instead: Min: %i, Max: %i\n", queryctrl.minimum, queryctrl.maximum);
      break;
    }
  }
}

void NaoCamera::displayAvailableSettings() {

  memset (&queryctrl, 0, sizeof (queryctrl));

  std::cout << "NaoCamera: Displaying Typical Parameters:-" << std::endl;

  for (queryctrl.id = V4L2_CID_BASE;
      queryctrl.id < V4L2_CID_LASTP1;
      queryctrl.id++) {
    if (0 == ioctl (videoDeviceFd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

      printf (" Control %s (%i)\n", queryctrl.name, queryctrl.id);

      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        enumerateSettingMenu();
      else
        printf ("  Min: %i, Max: %i\n", queryctrl.minimum, queryctrl.maximum);
    } else {
      if (errno != EINVAL)
        perror ("  Found parameter, but query still failed (VIDIOC_QUERYCTRL)");
    }
  }

  std::cout << "NaoCamera: Displaying Private Parameters:-" << std::endl;

  for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;
      queryctrl.id++) {
    if (0 == ioctl (videoDeviceFd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;

      printf (" Control %s\n", queryctrl.name);

      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        enumerateSettingMenu();
      else
        printf ("  Min: %i, Max: %i\n", queryctrl.minimum, queryctrl.maximum);
    } else {
      if (errno == EINVAL)
        break;
      perror ("  Found parameter, but query still failed (VIDIOC_QUERYCTRL)");
    }
  }

  std::cout << "NaoCamera: Displaying Camera Control Parameters:-" << std::endl;

  for (queryctrl.id = V4L2_CID_CAMERA_CLASS_BASE;
      queryctrl.id <= V4L2_CID_CAMERA_CLASS_BASE + 12;
      queryctrl.id++) {
    if (0 == ioctl (videoDeviceFd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        continue;
      printf (" Control %s\n", queryctrl.name);
      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        enumerateSettingMenu();
      else
        printf ("  Min: %i, Max: %i\n", queryctrl.minimum, queryctrl.maximum);
    } else {
      if (errno != EINVAL)
        perror ("  Found parameter, but query still failed (VIDIOC_QUERYCTRL)");
    }
  }

}

void NaoCamera::getCameraParams() {

  read_camera_params_.kCameraAutoWhiteBalance = getControlSetting(V4L2_CID_AUTO_WHITE_BALANCE);
  read_camera_params_.kCameraExposureAuto = getControlSetting(V4L2_CID_EXPOSURE_AUTO);
  read_camera_params_.kCameraBacklightCompensation = getControlSetting(V4L2_CID_BACKLIGHT_COMPENSATION);

  read_camera_params_.kCameraBrightness = getControlSetting(V4L2_CID_BRIGHTNESS);
  read_camera_params_.kCameraContrast = getControlSetting(V4L2_CID_CONTRAST);
  read_camera_params_.kCameraSaturation = getControlSetting(V4L2_CID_SATURATION);
  read_camera_params_.kCameraHue = getControlSetting(V4L2_CID_HUE);
  read_camera_params_.kCameraExposure = getControlSetting(V4L2_CID_EXPOSURE);
  read_camera_params_.kCameraGain = getControlSetting(V4L2_CID_GAIN);
  read_camera_params_.kCameraSharpness = getControlSetting(V4L2_CID_SHARPNESS);
}

void NaoCamera::updateBuffers() {
  dequeueBuffer();
  swapBuffers();
  dequeueBuffer();
  enqueueBuffer();
  swapBuffers();
}

void NaoCamera::enqueueBuffer() {
  if(prevBuf) {
    int result = ioctl(videoDeviceFd, VIDIOC_QBUF, prevBuf);
    if(result < 0) std::cout << "NaoCamera: Error queuing current buffer\n";
  }
}

void NaoCamera::dequeueBuffer() {
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  int result = ioctl(videoDeviceFd, VIDIOC_DQBUF, buf);
  if(result < 0) {
    std::cout << "NaoCamera: Error dequeuing buffer: " << errno << "\n";
  }

  *nextBuf = *buf;
  timeStamp = storedTimeStamp+1;
  storedTimeStamp++;
}

void NaoCamera::swapBuffers() {
  *prevBuf = *nextBuf;
  nextBuf->index = -1;
}

uint8_t* NaoCamera::getImage() {
  if (!nextBuf || nextBuf->index < 0) {
    std::cerr << "NaoCamera(FATAL): Image buffer requested when not ready. Call updateBuffer first!!" << std::endl;
    return NULL;
  }
  return static_cast<uint8_t*>(mem[nextBuf->index]);
}

unsigned int NaoCamera::getTimeStamp() const {
  return timeStamp;
}

int NaoCamera::getControlSetting(unsigned int id) {
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(videoDeviceFd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    std::cerr << "NaoCamera (GET): Query failed on " << queryctrl.name << "(" << id << ")\n";
    return -1;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    std::cerr << "NaoCamera: Query disabled on " << queryctrl.name << "(" << id << ")\n";
    return -1;
  }
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU){
    std::cerr << "NaoCamera: Query not supported on " << queryctrl.name << "(" << id << ")\n";
    return -1; // not supported
  }

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(videoDeviceFd, VIDIOC_G_CTRL, &control_s) < 0){
    std::cerr << "NaoCamera: Control value get failed on " << queryctrl.name << "(" << id << ")\n";
    return -1;
  }
  return control_s.value;
}

bool NaoCamera::setControlSetting(unsigned int id, int value) {
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(videoDeviceFd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    std::cerr << "NaoCamera (SET): Query failed on " << queryctrl.name << "(" << id << ")\n";
    return false;
  }
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    std::cerr << "NaoCamera: Query disabled on " << queryctrl.name << "(" << id << ")\n";
    return false; // not available
  }
  std::cout << "successfully queried " << queryctrl.name << ", type: " << queryctrl.type << ", default: " << queryctrl.default_value << ", step: " << queryctrl.step << ", flags: " << queryctrl.flags << "\n";
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
    std::cerr << "NaoCamera: Query not supported on " << queryctrl.name << "(" << id << ")\n";
    return false; // not supported
  }

  // clip value
  int oldval = value;
  if(value < queryctrl.minimum)
    value = queryctrl.minimum;
  if(value > queryctrl.maximum)
    value = queryctrl.maximum;

  if (value != oldval) {
    std::cout << "NaoCamera: Value outside range [" << queryctrl.minimum << "," << queryctrl.maximum << "], " << oldval << " on " << queryctrl.name << "(" << id << ") got cropped to " << value << std::endl;
  }

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if(ioctl(videoDeviceFd, VIDIOC_S_CTRL, &control_s) < 0){
    std::cerr << "NaoCamera: Error setting " << queryctrl.name << "(" << id << ") to " << value << "\n";
    return false;
  }
  int newvalue = getControlSetting(id);
  if(newvalue != value) {
    std::cerr << "NaoCamera: " << queryctrl.name << "(" << id << ") requested as " << value << ", but was set to " << newvalue << "\n";
    return false;
  }
  std::cout << "NaoCamera: " << queryctrl.name << "(" << id << ") requested as " << value << " successfully\n";
  return true;
}
