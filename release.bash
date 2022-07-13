#!/bin/bash

BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function trace() {
    (echo >&2 -e "[${GREEN}${BOLD} TRACE L${BASH_LINENO[0]} (${FUNCNAME[1]})${NO_COLOR}] $*")
}

PROJECT_ROOT_DIR=$(pwd)
PROJECT_RELEASE_DIR=${PROJECT_ROOT_DIR}/output
rm -rf ${PROJECT_RELEASE_DIR}
if [[ ! -d ${PROJECT_RELEASE_DIR}/lib/ ]];then
    mkdir -p ${PROJECT_RELEASE_DIR}/lib/
fi
cp -R -f ${PROJECT_ROOT_DIR}/bin ${PROJECT_RELEASE_DIR}/bin
cp -R -f ${PROJECT_ROOT_DIR}/config ${PROJECT_RELEASE_DIR}/config
trace "Copyed binary and configuration files"

# example
# awk '{print $3}' Print Col 3
# grep -v ${PROJECT_RELEASE_DIR}/output remove line with xxx
# xargs -i send to {}
# cp -n copy when not exist
# ldd -r ${PROJECT_RELEASE_DIR}/bin/vis | awk '{print $3}' | grep -v ${PROJECT_RELEASE_DIR}/output | xargs -i cp -n {} ${PROJECT_RELEASE_DIR}/output/lib/

ldd -r ${PROJECT_RELEASE_DIR}/bin/vis | awk '{print $3}' | xargs -i cp -n {} ${PROJECT_RELEASE_DIR}/lib/
ldd -r ${PROJECT_RELEASE_DIR}/bin/pre | awk '{print $3}' | xargs -i cp -n {} ${PROJECT_RELEASE_DIR}/lib/
trace "Copyed necessary libraries"

trace "Add rpath $ORIGIN of ${APOLLO_ROOT_DIR}/lib/*"
for file in ${PROJECT_RELEASE_DIR}/lib/*; do
    if test -f $file; then
        rpath=$(patchelf --print-rpath $file)
        patchelf --set-rpath '$ORIGIN':${rpath} $file
        rpath=$(patchelf --print-rpath $file)
    fi
done

trace "Add rpath $ORIGIN of ${APOLLO_ROOT_DIR}/output/bin/*"
for file in ${PROJECT_RELEASE_DIR}/bin/*; do
    if test -f $file; then
        rpath=$(patchelf --print-rpath $file)
        patchelf --set-rpath '$ORIGIN/../lib':${rpath} $file
        rpath=$(patchelf --print-rpath $file)
    fi
done