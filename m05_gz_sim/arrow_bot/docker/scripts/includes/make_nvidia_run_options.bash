    # If Nvidia is Prime (main video card) it will add Nvidia container options (must be installed nvidia-container-toolkit)
    # [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
    # If dont need this set Prime integrated video (f.e. sudo prime-select intel). It usually is default.
    nvidia_options=''
    if [ "$(prime-select query 2> /dev/null)" == 'nvidia' ]; then 
        nvidia_options='--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all'
    fi