FROM ghcr.io/ca-giken/rosnoetic-base:main

WORKDIR /root

# Install Eel and Chromium
RUN pip install eel
RUN apt-get update \
    && apt-get install -y libgtk2.0-0 libgtk-3-0 libnotify-dev libgconf-2-4 libnss3 libxss1 libasound2 libxtst6 xauth xvfb libgbm-dev fonts-ipafont \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Temporarily install rovi for catkin build
# Rovi setup(Aravis)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ORGE_RTT_MODE=Copy
RUN apt-get update -q && apt-get install -y \
    g++ automake intltool libgstreamer*-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
RUN tar -xvf aravis-0.6.0.tar.xz
RUN cd aravis-0.6.0 && ./configure && make && make install

# Rovi_industrial setup
RUN pip install git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git

# for ply-mesure-demo
RUN pip install --upgrade pip setuptools wheel
RUN pip install sip numpy
RUN pip install open3d==0.13.0
RUN add-apt-repository ppa:okirby/qt6-backports && \
    apt-get update -q && apt-get install -y \
    qt6-base-dev \
    libxcb-cursor0 libxcb1 libxcb-keysyms1 \
    libxcb-icccm4 libxcb-image0 libxcb-render-util0 libxcb-randr0 \
    libxcb-xinerama0 libxcb-xkb1 libxcb-shape0 libxkbcommon-x11-0\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN pip install PyQt6
COPY ./requirements.txt /root/requirements.txt
RUN pip install -r requirements.txt

COPY ./entrypoint.sh /root/entrypoint.sh
ENTRYPOINT ["/bin/bash", "-c", "/root/entrypoint.sh"]