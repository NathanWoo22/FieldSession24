docker run -it \
 -e DISPLAY=$DISPLAY \
 -v ./scripts:/scripts:z \
 -v ./datasets:/datasets:z \
  test:latest
