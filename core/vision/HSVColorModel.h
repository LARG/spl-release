#ifndef HSVCOLORMODEL_H
#define HSVCOLORMODEL_H

#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <common/ColorSpaces.h>

/** 
      Generates a color table using thresholds over HSV color space 
 
 
*/
class HSVColorModel {

  public:

    void createColorTable(unsigned char *colorTable);
    Color yuvToColor(int y, int u, int v); 


};


#endif
