XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi
xhost +local:root
#--net=allow using ros from host
docker run -it --name vslam --net=host --rm\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -v ~/work/openvslam/vocab:/vocab -v ~/work/openvslam/config:/config \
    --runtime=nvidia \
    openvslam \
    bash

#rosrun openvslam run_slam -v /vocab/orb_vocab.dbow2 -c /config/config.yaml --topic1 /stereo/left/image_raw --map-db /config/track6.msg
#apt-get install ros-melodic-rviz -y
