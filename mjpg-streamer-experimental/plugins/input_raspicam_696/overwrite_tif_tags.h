/**
 * @brief Overwrite the existing tiff headers with Team 696 bounding box
 *        information.
 *
 * @param cols [in]             The number of columns in the image.
 * @param rows [in]             The number of rows in the image.
 * @param bbox_coord_count [in] The number of elements in the bbox_coord array.
 *                              This is truncated down in this routine to fit
 *                              in the available space, and to be a multiple of
 *                              four.
 * @param bbox_coord [in]       The bounding box coordinates.  Each bounding
 *                              box requires 4 coordinates in this order:
 *                              x_min, y_min, x_max, y_max.
 * @param buf [in,out]          Points to the start of the JPEG header.  The
 *                              size of the header is encoded in the header
 *                              itself.  This routine writes over the TIF tags
 *                              portion of the header with the supplied
 *                              bounding box info.
 * @return On success, the number of bbox_coords actually written.  This may
 *         be less than bbox_coord.  On failure, -1.  Failure occurs if
 *         buf does not point to a valid JPEG header, or if there is not
 *         enough space in the header to hold the minimal set of TIF tags.
 */
int overwrite_tif_tags(unsigned int cols,
                       unsigned int rows,
                       unsigned short bbox_coord_count,
                       unsigned short bbox_coord[],
                       unsigned char buf[]);
