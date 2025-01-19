# based on custom base image with deps preinstalled, for the devs sanity
FROM algotour-native-deps:latest AS builder

RUN echo "------ Building AT native ------"
COPY src /app/src
COPY CMakeLists.txt /app/CMakeLists.txt
WORKDIR /app/build 
RUN cmake /app && cmake --build .

RUN cp /app/build/AlgotourNative /app/AlgotourNative
RUN rm -rf /app/build
RUN rm -rf /app/src

WORKDIR /app

ENTRYPOINT [ "/app/AlgotourNative", "--riskmap_path", "/riskmaps/rm.tif", "--dem_path", "/dem/dem.tif", "--port",  "50051" ]