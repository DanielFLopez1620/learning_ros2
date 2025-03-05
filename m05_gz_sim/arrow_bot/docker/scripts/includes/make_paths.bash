#!/bin/bash

# make path_to_ws
# catch last /src/ dir and take path
re="^(.*\/)src\/.*?$"
if [[ $run_script_dir =~ $re ]]; then
    path_to_ws=${BASH_REMATCH[1]}
else
    # write error message to stderr
    printf '%s\n' "ERROR: Can not detect ros workspace. Package must be somethere in ros workspace. \
    Also ensure you use bash shell." >&2
    exit 1
fi

# make path_to_git_root
# go up to 2 levels and take path
re="^(.*\/).*?\/.*?$"
if [[ $run_script_dir =~ $re ]]; then
    path_to_git_root=${BASH_REMATCH[1]}
else
    # write error message to stderr
    printf '%s\n' "ERROR: Can not detect git root. Generated directories must have original structure." >&2
    exit 1
fi

modules_path="$path_to_git_root/modules/"
modules_replacements_path="$path_to_git_root/modules_replacements/"
# inside build_data_path path is mounted to host - build, install, logs, not_ros_packges dirs from container
build_data_path=$path_to_ws'build__'$package_name 
cont_user_path=/home/$user_in_cont
cont_path_ws=$cont_user_path/$path_to_ws_relatively_user_in_cont


echo 'Vars: '
echo '$run_script_dir: '$run_script_dir
echo '$path_to_ws: '$path_to_ws
echo '$cont_path_ws: ' $cont_path_ws
echo '$build_data_path: '$build_data_path
echo '$path_to_git_root: '$path_to_git_root
echo '$modules_path: '$modules_path
echo '$modules_replacements_path: '$modules_replacements_path
echo ''
