gcc -o detect_color_blobs -I .. -g detect_color_blobs_main.c ../detect_color_blobs.c -ljpeg
gcc -o yuv_color_space_image -I .. -g yuv_color_space_image_main.c yuv_color_space_image.c ../yuv420.c
gcc -o convert_yuv_to_jpg -I .. -g convert_yuv_to_jpg.c ../yuv420.c jpeg_file_io.c -ljpeg
