# Use an official Ubuntu as a parent image
FROM ubuntu:latest

# Set the maintainer label
LABEL maintainer="jesse@born4life.ch"

# Install dependencies
RUN apt-get update && apt-get install -y \
  git \
  cmake \
  build-essential \
  autoconf \
  libtool \
  pkg-config \
  gdal-bin \
  libprotobuf-dev \
  libgdal-dev \
  libabsl-dev \
  wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*


RUN echo "------ Building UUIDgen ------"
WORKDIR /usr/utillinux
RUN apt-get update && apt-get install -y wget
RUN wget https://www.kernel.org/pub/linux/utils/util-linux/v2.40/util-linux-2.40.tar.gz
RUN tar -xzf util-linux-2.40.tar.gz
RUN pwd
WORKDIR /usr/utillinux/util-linux-2.40
RUN ./configure --disable-all-programs --enable-libuuid
RUN make libuuid
RUN make install

RUN ls /usr/include/uuid

RUN echo "------ Building grpc ------"

RUN ldconfig /usr/include/uuid

# Set environment variables for GDAL
# ENV CPLUS_INCLUDE_PATH=/usr/include/gdal
# ENV C_INCLUDE_PATH=/usr/include/gdal
RUN mkdir -p /usr/grpcbuild/
WORKDIR /usr/grpcbuild/
ENV GRPC_DIR=/usr/local
RUN mkdir -p $GRPC_DIR
ENV PATH="$GRPC_DIR/bin:$PATH"
RUN git clone --recurse-submodules -b v1.66.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc
RUN cd grpc
RUN mkdir -p cmake/build
WORKDIR /usr/grpcbuild/grpc/cmake/build
RUN pwd
RUN ls
RUN cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX=$GRPC_DIR -DgRPC_ABSL_PROVIDER=module ../..
RUN make -j 8
RUN make install
RUN cd ../..

RUN ldconfig

RUN echo "------ Building AT native ------"
COPY src /app/src
COPY CMakeLists.txt /app/CMakeLists.txt
WORKDIR /app
RUN mkdir -p /app/build 
WORKDIR /app/build 
RUN cmake /app
RUN cmake --build .