xhost local:docker
docker run -it \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v ~/.Xauthority:/root/.Xauthority:ro \
 -v ./scripts:/scripts:z \
 -v ./datasets:/datasets:z \
 -v ./read_pcd.py:/read_pcd.py \
 -v ./icp_registration.py:/icp_registration.py \
 -v ./icp_algo.py:/icp_algo.py \
 -v ./convert_pcd.py:/convert_pcd.py \
 --net=host \
 --rm \
 --cap-add=NET_ADMIN \
 fieldsession:latest