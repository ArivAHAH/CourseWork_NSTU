FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential cmake pkg-config \
    qt6-base-dev qt6-tools-dev qt6-tools-dev-tools \
    qt6-webengine-dev \
    qt6-webengine-dev-tools \
    libqt6webenginecore6 \
    libqt6webenginewidgets6 \
    libqt6webenginecore6-bin \
    libqt6webengine6-data \
    libqt6sql6-psql \
    libbox2d-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    mesa-common-dev \
    libx11-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxrender-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j

CMD ["./build/traffic-sim"]
