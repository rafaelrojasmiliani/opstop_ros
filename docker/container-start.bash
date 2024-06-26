main(){



    scriptdir=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
    if [ "$scriptdir" != "$(pwd)" ]; then
      echo "this script must be executed from $scriptdir".
      exit 1
    fi


    if lspci | grep -qi "vga .*nvidia" && \
        docker -D info 2>/dev/null | grep -qi "runtimes.* nvidia"; then
        DOCKER_NVIDIA_OPTIONS="\
            --runtime=nvidia \
            --env=NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
            --env=NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}all"
    else
        DOCKER_NVIDIA_OPTIONS=""
    fi

    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    DOCKER_VIDEO_OPTIONS="${DOCKER_NVIDIA_OPTIONS} --env=DISPLAY --env=QT_X11_NO_MITSHM=1 --env=XAUTHORITY=$XAUTH --volume=$XAUTH:$XAUTH --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"

    OPTIND=1

    options=$(getopt -o '' --long "network: " -- "$@")

    eval set -- "$options"

    [ $? -eq 0 ] || {
            echo "Incorrect options provided"
        exit 1
    }

    if [[ $1 != "--" ]] && [[ $1=="--network" ]]; then
        shift
        if [[ $1 == "smf" ]]; then

        DOCKER_NETWORK_OPTIONS="--net=br0 --env=ROS_MASTER_URI=http://10.10.238.1:11311 --env=ROS_IP=10.10.238.5 --ip 10.10.238.5 "

        if [[ ! $(docker network ls | grep br0 ) ]]; then
            docker network create --driver=bridge --ip-range=10.10.238.0/24 --subnet=10.10.238.0/24 --aux-address='ip1=10.10.238.4' -o "com.docker.network.bridge.name=br0" br0
        fi
    fi
	    if [[ $1 == "fhi" ]]; then

		    DOCKER_NETWORK_OPTIONS="--net=br0 --env=ROS_MASTER_URI=http://172.16.0.203:11311 --env=ROS_IP=172.16.0.202 --ip 172.16.0.202"

		    if [[ ! $(docker network ls | /bin/grep br0 ) ]]; then
			    docker network create --driver=bridge --ip-range=172.16.0.0/24 --subnet=172.16.0.0/24 --aux-address='ip1=172.16.0.201' -o "com.docker.network.bridge.name=br0" br0
		    fi
	    fi
    else

        DOCKER_NETWORK_OPTIONS="--env=ROS_MASTER_URI=http://127.0.0.1:11311 --env=ROS_MASTER_IP=127.0.0.1 --env=ROS_IP=127.0.0.1"
    fi

    docker run -it --rm \
        ${DOCKER_VIDEO_OPTIONS} \
        ${DOCKER_NETWORK_OPTIONS} \
        --privileged \
        --volume $(pwd)/../:/catkinws/src/ \
        --user $(id -u):$(id -g) \
        opstop_ros bash
}

main $@
