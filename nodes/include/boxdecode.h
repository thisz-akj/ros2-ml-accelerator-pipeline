/**************************************************************************/
//||                        SiMa.ai CONFIDENTIAL                          ||
//||   Unpublished Copyright (c) 2022-2024 SiMa.ai, All Rights Reserved.  ||
//**************************************************************************
// NOTICE:  All information contained herein is, and remains the property of
// SiMa.ai. The intellectual and technical concepts contained herein are 
// proprietary to SiMa and may be covered by U.S. and Foreign Patents, 
// patents in process, and are protected by trade secret or copyright law.
//
// Dissemination of this information or reproduction of this material is 
// strictly forbidden unless prior written permission is obtained from 
// SiMa.ai.  Access to the source code contained herein is hereby forbidden
// to anyone except current SiMa.ai employees, managers or contractors who 
// have executed Confidentiality and Non-disclosure agreements explicitly 
// covering such access.
//
// The copyright notice above does not evidence any actual or intended 
// publication or disclosure  of  this source code, which includes information
// that is confidential and/or proprietary, and is a trade secret, of SiMa.ai.
//
// ANY REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC PERFORMANCE, OR PUBLIC
// DISPLAY OF OR THROUGH USE OF THIS SOURCE CODE WITHOUT THE EXPRESS WRITTEN
// CONSENT OF SiMa.ai IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE 
// LAWS AND INTERNATIONAL TREATIES. THE RECEIPT OR POSSESSION OF THIS SOURCE
// CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS TO 
// REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR
// SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.                
//
/**************************************************************************/

#ifndef BOXDECODE_H
#define BOXDECODE_H

#include <vector>

typedef int32_t DimSizeType;
const int mask_size_x = 160;
const int mask_size_y = 160;
// format of data for downstream plugins
struct BoundingBoxOut { // output of YOLO-like one-stage box decode
  unsigned int _x;
  unsigned int _y;
  unsigned int _w;
  unsigned int _h;
  float _score;
  unsigned int _class_id;
  void print() {std::cout << "Box coord: x,y,w,h, score, class " << _x << "," << _y << "," << _w << "," << _h << "," << _score 
     << "," << _class_id << "\n"; }
};

#define MAX_NUM_POSE_POINTS 17
struct PoseOut {
  struct PosePoint {
    uint32_t _x;
    uint32_t _y;
    float _visible;
  } pose_points[MAX_NUM_POSE_POINTS];
};
#define MASK_PROTO_LENGTH 32
#define MAX_NUMBER_MASKS 20

struct BoundingBoxWithMaskOut : public BoundingBoxOut {
  float mask_prototype[MASK_PROTO_LENGTH];
};
struct Output_RCNN_S1 { // output of RCNN Stage 1 box decode
  float16_t score;
  float16_t label;
  float16_t x1;
  float16_t y1;
  float16_t x2;
  float16_t y2;
};

extern "C" int configure(int instance, const char * json_file_name);


#endif
