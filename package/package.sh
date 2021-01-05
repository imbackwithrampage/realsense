#!/usr/bin/env bash
##
## Copyright (c) 2020 Hanson Robotics.
##
## This file is part of Hanson AI.
## See https://www.hansonrobotics.com/hanson-ai for further info.
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##

package() {
    local reponame=realsense

    mkdir -p $BASEDIR/src
    rsync -r --delete \
        --exclude ".git" \
        --exclude "package" \
        $BASEDIR/../ $BASEDIR/src/$reponame

    get_version $1
    source_ros
    catkin_make_isolated --directory $BASEDIR --install --install-space $BASEDIR/install -DCMAKE_BUILD_TYPE=Release

    local name=head-ros-realsense
    local desc="ROS Wrapper for Intel® RealSense™ Devices"
    local url="https://api.github.com/repos/hansonrobotics/$reponame/releases"

    if [[ $ROS_PYTHON_VERSION == 2 ]]; then
        error "This package does not support python2"
        exit 1
    fi

    fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
        --url "${url}" --description "${desc}" ${ms} --force \
        --deb-no-default-config-files \
        -p $BASEDIR/${name}_VERSION_ARCH.deb \
        install/share=${HR_ROS_PREFIX}/ \
        install/lib=${HR_ROS_PREFIX}/ \
        install/include=${HR_ROS_PREFIX}/

    cleanup_ros_package_build $BASEDIR
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    set -e

    package $1
fi
