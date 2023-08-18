########################################################################################################################
# Flex (simulation engine) build stage
########################################################################################################################
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04 AS build
MAINTAINER Satyalab, satya-group@lists.andrew.cmu.edu

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update 

RUN apt-get -y install \
    libzmq3-dev \
    protobuf-compiler libprotobuf-dev \
    libegl1-mesa-dev libgl1-mesa-dev \
    freeglut3-dev \
    libjpeg-dev 

# Download cuda toolkit 9.2, and place the /usr/local/cuda-9.2 in server/cuda-9.2 first
# https://developer.nvidia.com/cuda-92-download-archive
COPY server/ ./server/
   
RUN cd server && \
    make clean && \
    make release


########################################################################################################################
# openfluid server image
########################################################################################################################
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04
LABEL org.opencontainers.image.source=https://github.com/cmusatyalab/openfluid
LABEL org.opencontainers.image.description="OpenFluid: Real-Time Fluid Simulation run file. For Nvidia driver version 470"
LABEL org.opencontainers.image.licenses="Apache License, Version 2.0"

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update 

# Library for Flex simulation engine
RUN apt-get -y install \
    libzmq3-dev \
    libprotobuf-dev \
    libegl1-mesa-dev libgl1-mesa-dev libglu1-mesa-dev \
    freeglut3-dev \
    libjpeg-dev \
    software-properties-common

# need to match the Server Driver version
RUN apt-get -y install \
    # libnvidia-gl-535     
    #libnvidia-gl-530
    libnvidia-gl-525
    #libnvidia-gl-520
    #libnvidia-gl-510
    #libnvidia-gl-495
    # libnvidia-gl-470

# Python Server Requirement
RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update && apt-get install -y \
    python3.8 \
    python3-pip \
 && apt-get install -y --reinstall python3.8-distutils

RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY ./server/run-requirements.txt requirements.txt

RUN python3.8 -m pip install --upgrade pip \
 && python3.8 -m pip install --no-cache-dir \
    -r /requirements.txt

# copy built Binary, python files, and data file
COPY --from=build \
    /server/*.py /server/entrypoint.sh /server/

# COPY --from=build \
#     /server/cuda-9.2/lib64/ /server/cuda-9.2/lib64/

COPY --from=build \
    /server/Flex/external/SDL2-2.0.4/lib/ /server/Flex/external/SDL2-2.0.4/lib/

COPY --from=build \
    /server/Flex/bin/ /server/Flex/bin/

COPY --from=build \
    /server/Flex/data/ /server/Flex/data/

WORKDIR /server

EXPOSE 9099
ENTRYPOINT ["./entrypoint.sh"]