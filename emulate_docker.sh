#!/bin/env bash

docker build -t dreame_z10_emu .

rm emulation.log

docker run \
   -v ./tmp:/tmp \
   -v ./helpers:/helpers \
   -v ./overrides/p2028.conf:/ava/conf/p2028.conf \
   -it \
   --rm \
   dreame_z10_emu\
   /bin/sh -c "/helpers/prepare_env.sh; QEMU_STRACE=1 /usr/bin/ava -f /ava/conf/p2028.conf force" 2>&1 | tee emulation.log
