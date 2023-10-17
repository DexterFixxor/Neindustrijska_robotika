#!/bin/bash
set -e

cd ${WS_DIR}
. install/setup.bash

# Run the main container process (from the Dockerfile CMD for example)
exec "$@"