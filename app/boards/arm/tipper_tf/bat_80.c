#include "lvgl.h"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif
#ifndef LV_ATTRIBUTE_IMG_BAT_80
#define LV_ATTRIBUTE_IMG_BAT_80
#endif
const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_IMG_BAT_80 uint8_t bat_80_map[] = {
  0xfb, 0xfb, 0xfb, 0xff, 	/*Color of index 0*/
  0x02, 0x02, 0x02, 0xff, 	/*Color of index 1*/

  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf2, 0x7c, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x3c, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xce, 0x7c, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x9c, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x39, 0xfc, 
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x73, 0xf8, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x27, 0xf0, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xe0, 
  0xff, 0xff, 0xff, 0xff, 0xf3, 0xdf, 0xc0, 
  0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 0x80, 
  0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0x00, 
  0xff, 0xff, 0xff, 0xff, 0x80, 0xfe, 0x00, 
  0xff, 0xff, 0xff, 0xff, 0x01, 0xfc, 0x00, 
  0xff, 0xff, 0xff, 0xfe, 0x03, 0xf8, 0x00, 
  0xff, 0xff, 0xff, 0xfc, 0x07, 0xf0, 0x00, 
  0xff, 0xff, 0xff, 0xfc, 0x0f, 0xe0, 0x00, 
  0xff, 0xff, 0xff, 0xfe, 0x1f, 0xc0, 0x00, 
  0xff, 0xff, 0xff, 0xef, 0x3f, 0x80, 0x00, 
  0xff, 0xff, 0xff, 0xc7, 0xff, 0x00, 0x00, 
  0xff, 0xff, 0xff, 0x83, 0xfe, 0x00, 0x00, 
  0xff, 0xff, 0xff, 0x01, 0xfc, 0x00, 0x00, 
  0xff, 0xff, 0xfe, 0x03, 0xf8, 0x00, 0x00, 
  0xff, 0xff, 0xfc, 0x07, 0xf0, 0x00, 0x00, 
  0xff, 0xff, 0xf8, 0x0f, 0xe0, 0x00, 0x00, 
  0xff, 0xff, 0xf0, 0x1f, 0xc0, 0x00, 0x00, 
  0xff, 0xff, 0xf8, 0x3f, 0x80, 0x00, 0x00, 
  0xff, 0xff, 0xfc, 0x7f, 0x00, 0x00, 0x00, 
  0xff, 0xff, 0x9e, 0xfe, 0x00, 0x00, 0x04, 
  0xff, 0xff, 0x0f, 0xfc, 0x00, 0x00, 0x0c, 
  0xff, 0xfe, 0x07, 0xf8, 0x00, 0x00, 0x1c, 
  0xff, 0xfc, 0x07, 0xf0, 0x00, 0x00, 0x3c, 
  0xff, 0xf8, 0x0f, 0xe0, 0x00, 0x00, 0x7c, 
  0xff, 0xf0, 0x1f, 0xc0, 0x00, 0x00, 0xfc, 
  0xff, 0xe0, 0x3f, 0x80, 0x00, 0x01, 0xfc, 
  0xff, 0xe0, 0x7f, 0x00, 0x00, 0x03, 0xf8, 
  0xff, 0xf0, 0xfe, 0x00, 0x00, 0x07, 0xe0, 
  0xff, 0x79, 0xfc, 0x00, 0x00, 0x0f, 0xe0, 
  0xfe, 0x3f, 0xf8, 0x00, 0x00, 0x0f, 0x8c, 
  0xfc, 0x1f, 0xf0, 0x00, 0x00, 0x07, 0x1c, 
  0xf8, 0x0f, 0xe0, 0x00, 0x00, 0x83, 0x1c, 
  0xf0, 0x1f, 0xc0, 0x00, 0x00, 0xc0, 0x7c, 
  0xe0, 0x3f, 0x80, 0x00, 0x01, 0xe0, 0xfc, 
  0xc0, 0x7f, 0x00, 0x00, 0x03, 0xf0, 0xfc, 
  0x80, 0xfe, 0x00, 0x00, 0x07, 0xe0, 0xfc, 
  0xc1, 0xfc, 0x00, 0x00, 0x0f, 0xe0, 0x7c, 
  0xe3, 0xf8, 0x00, 0x00, 0x0f, 0x8c, 0x3c, 
  0xf7, 0xf0, 0x00, 0x00, 0x07, 0x1c, 0x3c, 
  0xff, 0xe0, 0x00, 0x00, 0x07, 0x1e, 0x1c, 
  0xff, 0xc0, 0x00, 0x00, 0xc0, 0x7f, 0x0c, 
};

const lv_img_dsc_t bat_80 = {
  .header.always_zero = 0,
  .header.w = 54,
  .header.h = 54,
  .data_size = 387,
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .data = bat_80_map,
};
