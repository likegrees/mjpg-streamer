static unsigned char limit(float in) {
    int rounded = (int)(in + 0.5);
    if (rounded < 0) return 0;
    if (rounded > 255) return 255;
    return (unsigned char) rounded;
}

void yuv_color_space_image(const unsigned int cols,
                           const unsigned int rows,
                           const unsigned char y,
                           unsigned char* yuv) {
    unsigned int ii;
    unsigned int jj;
    const unsigned int pixels_per_side = (cols < rows) ? cols : rows;

    /* Set Y values to y in a pixels_per_side x pixels_per_side square.
       The rest of the Y image is 0. */

    for (ii = 0; ii < pixels_per_side; ++ii) {
        int y_offset = ii * cols;
        unsigned char* y_row = &yuv[y_offset];
        for (jj = 0; jj < pixels_per_side; ++jj) {
            y_row[jj] = y;
        }
        while (jj < cols) {
            y_row[jj] = 0;
            ++jj;
        }
    }
    while (ii < rows) {
        int y_offset = ii * cols;
        unsigned char* y_row = &yuv[y_offset];
        for (jj = 0; jj < cols; ++jj) {
            y_row[jj] = 0;
        }
        ++ii;
    }


    /* Set U and V values.  U values run 0 .. 255 increasing left to right.
       V values run 0 .. 255 increasing bottom to top.   Anything outside of
       pixels_per_side x pixels_per_side square is set to (128, 128). */

    unsigned int ii2;
    unsigned int jj2;
    float steps_per_pixel = 256.0 / pixels_per_side;
    for (ii2 = 0; ii2 < pixels_per_side / 2; ++ii2) {
        int u_offset = cols * rows + ii2 * (cols / 2);
        int v_offset = u_offset + cols * rows / 4;
        unsigned char* u_row = &yuv[u_offset];
        unsigned char* v_row = &yuv[v_offset];
        for (jj2 = 0; jj2 < pixels_per_side / 2; ++jj2) {
            u_row[jj2] = limit(2 * jj2 * steps_per_pixel);
            v_row[jj2] = limit((pixels_per_side - 2 * ii2) * steps_per_pixel);
        }
        while (jj2 < cols / 2) {
            u_row[jj2] = 128;
            v_row[jj2] = 128;
            ++jj2;
        }
    }
    while (ii2 < rows / 2) {
        int u_offset = cols * rows + ii2 * (cols / 2);
        int v_offset = u_offset + cols * rows / 4;
        unsigned char* u_row = &yuv[u_offset];
        unsigned char* v_row = &yuv[v_offset];
        for (jj2 = 0; jj2 < cols / 2; ++jj2) {
            u_row[jj2] = 128;
            v_row[jj2] = 128;
        }
        ++ii2;
    }
}
