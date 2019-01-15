
#include <vision/HSVColorModel.h>



void HSVColorModel::createColorTable(unsigned char *colorTable){

  for (int y = 0; y <= 255; y++){
    for (int u = 0; u <= 255; u++){
      for (int v = 0; v <= 255; v++){
        ColorTableMethods::assignColor(colorTable, y,u,v, yuvToColor(y,u,v));
      }
    }
  }
}


// Main function that converts a yuv pixel to a Color label
Color HSVColorModel::yuvToColor(int yy, int uu, int vv){

  Color c;
  RGB rgb = YUV444_TO_RGB(yy, uu, vv);
  HSV hsv = RGB_TO_HSV(rgb);


  if (hsv.v >= 0.90 && hsv.s <= 0.5 && (hsv.h < 0.2 || hsv.h > 0.5)){
    c = c_WHITE;            // This lets very bright stuff be called white. Kind of risky. Don't allow it for green since we want to distinguish lines. Hopefully lines don't get green hues... 
  }
  else if (hsv.s <= 0.25 && hsv.v >= 0.8){
    c = c_WHITE;            
  }
  else if (hsv.s <= 0.25){   // 0.25

    if (hsv.v >= 0.4)    // 0.65
      c = c_ROBOT_WHITE;
    else if (hsv.v < 0.4) // else
      c = c_BLUE;
  }
  else if (hsv.h >= 0.20 && hsv.h <= 0.5){ // .25
    c = c_FIELD_GREEN;
  }
  else {
    c = c_UNDEFINED;
  }

  return c;

}




/**

  ROBOCUP 2018

  We expanded the hue range of green to begin at 0.15 (includes more yellow), since the carpet was speckled. Note that we only used it for the outdoor field (1 game). 


  US OPEN 2018

  // At the US Open 2018, we used (with a manual WB of 3500):
  // We had to move the green before gray/black because of the drop in saturation in the camera... 

  if (hsv.v >= 0.90 && hsv.s <= 0.5 && (hsv.h < 0.15 || hsv.h > 0.55)){
  c = c_WHITE;            // This lets very bright stuff be called white. Kind of risky. Don't allow it for green since we want to distinguish lines. Hopefully lines don't get green hues... 
  }
  else if (hsv.s <= 0.10 && hsv.v >= 0.7 && (hsv.h < 0.15 || hsv.h > 0.55)){
  c = c_WHITE;            
  }
  else if (hsv.h >= 0.15 && hsv.h <= 0.55 && hsv.s >= 0.3){ // .25
  c = c_FIELD_GREEN;
  }
  else if (hsv.s <= 0.3 && hsv.v >= 0.8){
  c = c_WHITE;            
  }
  else if (hsv.s <= 0.3){   // 0.25
  if (hsv.v >= 0.4)    // 0.65
  c = c_ROBOT_WHITE;
  else if (hsv.v < 0.4) // else
  c = c_BLUE;
  }
  else {
  c = c_UNDEFINED;
  }


*/





