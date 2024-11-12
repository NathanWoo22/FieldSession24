# Sample run: bash run_compose.sh ./docker-compose.yaml icp-pcd

compose_filepath=$1
container_name=icp-pcd

control_c() {
  docker compose -f $compose_filepath down
  exit 1
}

echo "Starting $container_name container"
xhost +local:docker >> /dev/null

# Starting docker container
docker compose -f $compose_filepath up -d
trap 'control_c' SIGINT

#DO stuff
docker exec $container_name rviz2

# Stop docker container
docker compose -f $compose_filepath down
echo "Done"