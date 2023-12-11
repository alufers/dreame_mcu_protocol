FROM alpine:latest AS builder

WORKDIR /work

COPY nand0p5_dreame /work/nand0p5_dreame

RUN apk add --no-cache bash squashfs-tools git alpine-sdk autoconf autoconf automake binutils-dev elfutils-dev gawk linux-headers



RUN mkdir -p /work/rootfs
RUN unsquashfs -d /work/rootfs /work/nand0p5_dreame

# FROM dockcross/linux-aarch64:latest

# RUN git clone https://github.com/strace/strace && \
#     cd strace && \
#     ./bootstrap && \
#     LDFLAGS='-static -pthread' ./configure && \
#     LDFLAGS='-static -pthread' make

FROM scratch
COPY --from=builder /work/rootfs /
CMD ["/bin/sh"]
