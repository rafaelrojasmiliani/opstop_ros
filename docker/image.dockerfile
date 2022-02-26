# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros_noetic_vim

RUN apt clean
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    python3-pip git iputils-ping net-tools netcat screen build-essential lsb-release gnupg2 curl less \
                    python3-sympy coinor-libipopt-dev sudo valgrind \
                    build-essential pkg-config git \
                    liblapack-dev liblapack3 libopenblas-base libopenblas-dev \
                    libgfortran-7-dev cmake libgsl-dev gdb python3-tk libeigen3-dev \
                    libboost-math-dev \
    && pip3 install setuptools matplotlib Mosek scipy quadpy six cython tk \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    ros-noetic-ifopt exuberant-ctags ros-noetic-ros-base python3-catkin-tools \
                    ros-noetic-plotjuggler \
                    ros-noetic-joint-trajectory-controller \
                    ros-noetic-joint-trajectory-action \
                    ros-noetic-xacro \
                    ros-noetic-gazebo-ros \
                    ros-noetic-gazebo-ros-control \
                    ros-noetic-joint-state-controller \
                    ros-noetic-position-controllers \
                    ros-noetic-robot-state-publisher \
                    ros-noetic-joint-state-publisher \
                    ros-noetic-rqt \
                    ros-noetic-rqt-graph \
                    ros-noetic-roslint \
                    ros-noetic-plotjuggler-ros \
                    ros-noetic-rqt-gui \
                    ros-noetic-rqt-gui-py \
                    ros-noetic-rqt-py-common \
                    ros-noetic-moveit-msgs \
                    ros-noetic-rqt-joint-trajectory-controller \
                    ros-noetic-jsk-rviz-plugins \
    && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -


RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    robotpkg-py38-pinocchio \
    && echo "export PATH=/opt/openrobots/bin:$PATH" >> /etc/bash.bashrc \
    && echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH" >> /etc/bash.bashrc \
    && echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH" >> /etc/bash.bashrc \
    && echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH" >> /etc/bash.bashrc \
    && echo "export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH" >> /etc/bash.bashrc

RUN mkdir -p /aux_ws/src
RUN git clone https://github.com/rafaelrojasmiliani/ur_description_minimal.git /aux_ws/src/ur_description_minimal
RUN git clone https://github.com/tork-a/rqt_joint_trajectory_plot.git /aux_ws/src/rqt_joint_trajectory_plot
RUN bash -c 'source /opt/ros/noetic/setup.bash && cd /aux_ws && catkin config --install --install-space /opt/ros/noetic/ --extend /opt/ros/noetic/ && catkin build'


# user handling
ARG myuser
ARG myuid
ARG mygroup
ARG mygid
ARG scriptdir
RUN addgroup --gid ${mygid} ${mygroup} --force-badname
RUN adduser --gecos "" --disabled-password  --uid ${myuid} --gid ${mygid} ${myuser} --force-badname
#add user to sudoers
RUN echo "${myuser} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN echo "${myuser}:docker" | chpasswd


RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROSCONSOLE_FORMAT='[${severity}] - ${node}: [${time}] ${message}'" >> /etc/bash.bashrc
WORKDIR /catkinws
RUN chmod 777 /catkinws


RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --install-recommends -o Dpkg::Options::="--force-confnew" \
                    avahi-daemon \
                    avahi-autoipd \
                    openssh-server \
                    isc-dhcp-client \
                    iproute2

RUN service ssh start
RUN echo '[server]' >> /etc/avahi/avahi-daemon.conf
RUN echo 'enable-dbus=no' >> /etc/avahi/avahi-daemon.conf
RUN echo 'domain-name=local' >> /etc/avahi/avahi-daemon.conf
RUN echo 'host-name=gsplines-ros' >> /etc/avahi/avahi-daemon.conf
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
        gfortran libmetis-dev
RUN git clone https://github.com/coin-or-tools/ThirdParty-HSL.git /hsl
COPY coinhsl /hsl/coinhsl
COPY coinhsl /hsl/coinhsl
RUN cd /hsl && ./configure && make && make install
RUN ls /usr/local/lib
RUN cp $(find /usr/local/lib -name 'libcoinhsl*.so*' -type f) /usr/local/lib/libhsl.so
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> /etc/bash.bashrc
