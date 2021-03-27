#include "ST7920_I2C_MCP23017_GLCD.h"

ST7920_I2C_MCP23017_GLCD GLCD;

void setup() {
  //GLCD.set_inverse(true);
  //GLCD.set_rotate_180(true);
  GLCD.init_display(0x27, 0);
}

void loop() {
//  int wait = 15000;
//  bool inversed;
//  for(int i = 0; i < 2; i++) {
//    if(i == 0) {
//      inversed = false;
//    } else {
//      inversed = true;
//    }
//    GLCD.set_inverse(inversed);
//    GLCD.clear_screen();
//    GLCD.draw_line(0, 0, 127, 63);
//    GLCD.draw_line(127, 0, 0, 63);
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_triangle(32, 1, 1, 62, 62, 62);
//    GLCD.draw_filled_triangle(96, 1, 65, 62, 126, 62);
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_frame(1, 1, 62, 62);
//    GLCD.draw_box(65, 1, 62, 62);
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_rounded_frame(1, 1, 62, 62, 10);
//    GLCD.draw_filled_rounded_box(65, 1, 62, 62, 10);
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_circle(32, 32, 31);
//    GLCD.draw_filled_circle(96, 32, 31);
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_ellipse(32, 16, 62, 30);
//    GLCD.draw_filled_ellipse(96, 48, 62, 30);
//    delay(wait);
    GLCD.clear_screen();
    GLCD.write_string("Hello World) Not to try make long string");
//    ///glcd.set_cursour(20, 1);
//    GLCD.write_string("Сообщение на русском");
//    delay(wait);
//    GLCD.clear_screen();
//    draw_plot();
//    delay(wait);
//    GLCD.clear_screen();
//    GLCD.draw_bitmap(demo_bitmap, 128, 64);
//    delay(wait);
    delay(10000);
  //}
}

//void draw_plot() {
//  GLCD.write_char_x_y(4, 59, 0);
//  GLCD.write_gnumber_x_y(113, 59, 100);
//  GLCD.write_char_x_y(0, 0, 9);
//  GLCD.write_char_x_y(0, 53, 0);
//  GLCD.draw_line(4, 57, 128, 57);
//  GLCD.draw_line(4, 0, 4, 57);
//  int k = 32;
//  for (int i = 5; i < 128; i++) {
//    int j = rand() % 3;
//    GLCD.draw_pixel(i, k);
//    if (j == 1) {
//      k--;
//    }
//    if (j > 1) {
//      k++;
//    }
//  }
//}
