xhost local:docker
docker run -it \
 -e DISPLAY \
 -v ~/.Xauthority:/root/.Xauthority:ro \
 -v ./scripts:/scripts:z \
 -v ./datasets:/datasets:z \
 --net=host \
 --rm \
 --cap-add=NET_ADMIN \
  icp-pcd:latest
