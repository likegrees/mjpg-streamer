export LD_LIBRARY_PATH=.
./mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam_696.so -x 640 -y 480 -vwidth 320 -vheight 240 -fps 30 -quality 20 -ex auto -blobyuv 100,255,0,100,0,50 -ISO 100"

