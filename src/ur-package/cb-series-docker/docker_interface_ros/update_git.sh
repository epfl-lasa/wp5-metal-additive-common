#!/bin/bash

# Define the directory paths for the repositories
repo1_path="iiwa_toolkit"
repo2_path="iiwa_ros"


# Function to update a repository
update_repo() {
    local repo_path=$1
    cd "$repo_path"
    
    echo "Updating repository in $repo_path"
    git fetch
    git pull
    
    cd - >/dev/null
}

# Update all repositories
update_repo "$repo1_path"
update_repo "$repo2_path"


cd .. && catkin_make

echo "All repositories updated and catkin_make is done"
