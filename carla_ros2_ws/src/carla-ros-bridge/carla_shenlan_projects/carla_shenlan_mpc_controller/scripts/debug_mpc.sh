#!/usr/bin/env bash

set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
package_root="${workspace_root}/src/carla-ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller"

source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
set -u

exec gdb --args \
    "${workspace_root}/install/carla_shenlan_mpc_controller/lib/carla_shenlan_mpc_controller/mpc_lateral_longitudinal" \
    --ros-args \
    --params-file "${package_root}/config/mpc_parameters_configuration.yaml" \
    "$@"