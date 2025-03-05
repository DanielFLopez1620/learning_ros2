#!/bin/bash

max_try_git_submodule_update_or_add=10
sleep_interval_between_try=5


if [ -d "$path_to_git_root" ]; then
    cd $path_to_git_root

    gitmodules_init_file=.gitmodules_init
    # if modules init file exists do init and clone of git submodules from it and remove file after
    if [ -f "$gitmodules_init_file" ]; then
        echo 'Adding Git root submodules...'
        git config -f $gitmodules_init_file --get-regexp '^submodule\..*\.path$' |
            while read path_key path
            do
                name=$(echo $path_key | sed 's/submodule\.\(.*\)\.path/\1/')
                url_key=$(echo $path_key | sed 's/\.path/.url/') 
                branch_key=$(echo $path_key | sed 's/\.path/.branch/')
                url=$(git config -f $gitmodules_init_file --get "$url_key")
                branch=$(git config -f $gitmodules_init_file --get "$branch_key" || echo "master")


                loop_i=1
                # add submodule and if success remove it from $gitmodules_init_file
                until git submodule add --force -b $branch --name $name $url $path && git config -f $gitmodules_init_file --remove-section submodule.$name ; do
                    echo Git submodule add disrupted. Retring...

                    if [ "$loop_i" -gt "$max_try_git_submodule_update_or_add" ]; then
                        printf '%s\n' "ERROR: Max try for add git submodule reached. Exit." >&2
                        exit 1            
                    fi

                    ((loop_i++))
                    sleep $sleep_interval_between_try
                done


            done
        echo 'Git root submodules added.'

        if [ ! -s "$gitmodules_init_file" ]; then
            # $gitmodules_init_file is empty and can be remove
            rm -f $gitmodules_init_file
        fi
        
    fi
    
    echo 'Updating Git submodules...'
    loop_i=1
    # first update do init and update from superrepo recursively
    # second update do init and update from remote branch last commit only for root submodules
    until git submodule update --init --recursive && git submodule update --init --remote; do
        echo Git submodules update disrupted. Retring...

        if [ "$loop_i" -gt "$max_try_git_submodule_update_or_add" ]; then
            printf '%s\n' "ERROR: Max try for update git submodules reached." >&2
            exit 1            
        fi

        ((loop_i++))
        sleep $sleep_interval_between_try
    done
    echo 'Git submodules updated.'
    
else
    printf '%s\n' "ERROR: Bad path to git root." >&2
    exit 1
fi