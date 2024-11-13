xhost local:docker
docker run -it \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v ~/.Xauthority:/root/.Xauthority:ro \
 -v ./scripts:/scripts:z \
 -v ./datasets:/datasets:z \
 --net=host \
 --rm \
 --cap-add=NET_ADMIN \
 #icp-pcd:latest
  stratominc.azurecr.io/stratom_base:ubuntu22.04-humble-cuda
