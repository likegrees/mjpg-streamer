#include "overwrite_tif_tags.h"
int overwrite_tif_tags(unsigned int cols,
                      unsigned int rows,
                      unsigned short bbox_coord_count, 
                      unsigned short bbox_coord[],
                      unsigned char* buf) {

    /* Check the 1st four bytes to make sure this is a JPEG header. */

    if (buf[0] != 0xff ||
        buf[1] != 0xd8 ||
        buf[2] != 0xff ||
        buf[3] != 0xe1) return -1;

    /* Read the number of bytes in the remainder of the JPEG header from bytes
       4 and 5. */

    const unsigned int SIZE_FIELD_OFFSET = 4;
    unsigned int total_header_bytes = (((unsigned int)buf[4] << 8) | buf[5]) +
                                      SIZE_FIELD_OFFSET;

    /* Byte points to the start of the TIF tags.  All the offsets in the TIF
       tags portion of the header are indexed from here. */

    const unsigned int TIFF_TAGS_OFFSET = 12;
    unsigned char* byte = &buf[TIFF_TAGS_OFFSET];

    /* Byte count is the current space available in the TIF tags portion of
       the header. */

    unsigned short byte_count = total_header_bytes - TIFF_TAGS_OFFSET;

    const unsigned short BYTE_OFFSET_OF_BBOX_DATA = 50;
    const unsigned short BYTES_PER_COORD = sizeof(bbox_coord[0]);
    int max_data_bytes = byte_count - BYTE_OFFSET_OF_BBOX_DATA;
    if (max_data_bytes < 0) return -1;
    unsigned short max_coords = max_data_bytes / BYTES_PER_COORD;
    if (bbox_coord_count > max_coords) bbox_coord_count = max_coords;
    // Bbox_coord_count should be a multiple of 4.
    bbox_coord_count -= bbox_coord_count % 4;

    // Motorola byte order

    byte[0] = 'M';
    byte[1] = 'M';

    // TIF version 42

    byte[2] = 0x00;
    byte[3] = 0x2a;

    // Offset of start of IFD table

    byte[4] = 0x00;
    byte[5] = 0x00;
    byte[6] = 0x00;
    byte[7] = 0x08;

    // Count of IFDs

    byte[8] = 0x00;
    byte[9] = 0x03;

    // IFD 0: Columns in image

    byte[10] = 0x01;  // tag (2 bytes)
    byte[11] = 0x00;
    byte[12] = 0x00;  // type = unsigned ints (2 bytes)
    byte[13] = 0x04;
    byte[14] = 0x00;  // count of unsigned ints in data (4 bytes)
    byte[15] = 0x00;
    byte[16] = 0x00;
    byte[17] = 0x01;
    byte[18] = (cols & 0xff000000) >> 24;  // column count (4 bytes)
    byte[19] = (cols & 0x00ff0000) >> 16;
    byte[20] = (cols & 0x0000ff00) >> 8;
    byte[21] = (cols & 0x000000ff);

    // IFD 1: Rows in image

    byte[22] = 0x01;  // tag (2 bytes)
    byte[23] = 0x01;
    byte[24] = 0x00;  // type = unsigned ints (2 bytes)
    byte[25] = 0x04;
    byte[26] = 0x00;  // count of unsigned ints in data (4 bytes)
    byte[27] = 0x00;
    byte[28] = 0x00;
    byte[29] = 0x01;
    byte[30] = (rows & 0xff000000) >> 24;  // row count (4 bytes)
    byte[31] = (rows & 0x00ff0000) >> 16;
    byte[32] = (rows & 0x0000ff00) >> 8;
    byte[33] = (rows & 0x000000ff);

    // IFD 2: BBox data

    byte[34] = 0x96;  // tag (2 bytes)
    byte[35] = 0x96;
    byte[36] = 0x00;  // type = unsigned short (2 bytes)
    byte[37] = 0x03;
    byte[38] = 0x00;  // count of unsigned shorts in data (4 bytes)
    byte[39] = 0x00;
    byte[40] = (bbox_coord_count & 0xff00) >> 8;
    byte[41] = (bbox_coord_count & 0x00ff);
    byte[42] = 0x00;  // offset of start of bbox data (4 bytes)
    byte[43] = 0x00;
    byte[44] = 0x00;
    byte[45] = BYTE_OFFSET_OF_BBOX_DATA;

    // End of IFDs marker

    byte[46] = 0x00;
    byte[47] = 0x00;
    byte[48] = 0x00;
    byte[49] = 0x00;

    // Bbox coords

    unsigned short ii;
    unsigned char* p = &byte[BYTE_OFFSET_OF_BBOX_DATA];
    for (ii = 0; ii < bbox_coord_count; ++ii) {
        *(p++) = (bbox_coord[ii] & 0xff00) >> 8;
        *(p++) = (bbox_coord[ii] & 0x00ff);
    }

    // Clear unused bytes at end

    while (p < &byte[byte_count]) {
        *(p++) = 0;
    }
    return bbox_coord_count;
}
