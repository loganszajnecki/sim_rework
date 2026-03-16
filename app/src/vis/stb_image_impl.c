/* Compile stb_image in its own translation unit so that
   project warning flags do not apply to third-party code. */
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>