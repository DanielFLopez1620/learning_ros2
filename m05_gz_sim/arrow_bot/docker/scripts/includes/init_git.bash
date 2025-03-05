#!/bin/bash

echo 'Init Git...'
if [ -d "$path_to_git_root" ]; then
    cd $path_to_git_root

    if [ "$init_git" = true ] && [ ! -d '.git' ]; then
        git init -b main
    else
        if git status &>/dev/null; then
            echo "You choosed do not init Git and detected presence of Git. Will be used current git."
        else
            printf '%s\n' "ERROR: You choose do not init git. There is no git upper in directories. \
            Need any git for git submodules." >&2
            exit 1
        fi
    fi

    cd -
else
    printf '%s\n' "ERROR: Bad path to git root." >&2
    exit 1
fi