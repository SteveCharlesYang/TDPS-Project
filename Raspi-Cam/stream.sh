mjpg_streamer -i "input_uvc.so -d /dev/video0 -y -r 320x240 -f 60" -o "output_http.so -p 8964 -w ./cam"
