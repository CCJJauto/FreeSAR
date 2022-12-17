#!/usr/bin/env bash
MAZU_DEV_IMAGE=ubuntu
LOCAL_FREESAR_PATH=`pwd`
if ! docker pull "${MAZU_DEV_IMAGE}"; then
    error "Failed to pull docker image ${MAZU_DEV_IMAGE}, check locally"

    # check if the docker image exist locally
    check_result=`docker image ls ${MAZU_DEV_IMAGE} | wc -l`
    if [[ "$check_result" != "2" ]]; then
        error "Docker image ${MAZU_DEV_IMAGE} does not exist locally"
        exit 1
    else
        info "Docker image ${MAZU_DEV_IMAGE} exist locally"
    fi
fi
sudo docker run -itv "$LOCAL_FREESAR_PATH":/FreeSAR ubuntu
