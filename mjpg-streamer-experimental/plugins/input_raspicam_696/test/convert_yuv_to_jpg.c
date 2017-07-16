#include <stdio.h>
#include <stdlib.h>
#include "yuv420.h"
#include "jpeg_file_io.h"

int main(int argc, const char* argv[]) {
    int return_val = -1;
    if (argc != 2) {
        fprintf(stderr, "usage: convert_yuv_to_jpg in_file.yuv\n");
        return -1;
    }
    const char* in_filename = argv[1];
    Yuv_File yuv_file = yuv420_open_read(in_filename);
    if (yuv420_is_null(&yuv_file)) {
        fprintf(stderr, "can't open %s for reading\n", in_filename);
        return -1;
    }
    unsigned int cols = yuv420_get_cols(&yuv_file);
    unsigned int rows = yuv420_get_rows(&yuv_file);
    printf("cols=%u rows=%u\n", cols, rows);
    unsigned char* yuv = yuv420_malloc(&yuv_file);
    unsigned char* rgb = (unsigned char*)malloc(cols * rows * 3);
    int ii = 0;
    while (yuv420_read_next(&yuv_file, yuv) >= 0) {
        char filename[FILENAME_MAX];
        snprintf(filename, FILENAME_MAX, "out_%03d.jpg", ii);
        filename[FILENAME_MAX - 1] = '\0';
        convert_yuv420_to_rgb(cols, rows, yuv, rgb);
        if (jpeg_file_write(filename, 80, cols, rows, rgb) < 0) {
            fprintf(stderr, "can't write to %s\n", filename);
            goto quit;
        }
        ++ii;
    }
    return_val = 0;
quit:
    yuv420_close(&yuv_file);
    free(yuv);
    free(rgb);
    return return_val;
}
