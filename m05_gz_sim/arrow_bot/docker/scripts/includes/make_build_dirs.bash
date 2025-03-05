    # make dirs at host for builds, logs, other from container
    [ -d $build_data_path ] && rm -rf $build_data_path # remove dir
    for dir in 'build' 'install' 'log' 'ros2_system_logs' "$not_ros_packages_folder_name"
    do
        [ -d $build_data_path/$dir ] || mkdir -p $build_data_path/$dir # create dir
    done

    dir=$build_data_path/$not_ros_packages_folder_name
    echo "Remake $dir folder for clean bind cont -> host"
    [ -d $dir ] && rm -rf $dir && mkdir -p $dir