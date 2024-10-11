#!/bin/bash
# -*- coding: utf-8 -*-

# Script to help setup git hooks
# Author: Louis Munier - <lmunier@protonmail.com>
# Date: 2024-10-09

# Get the directory where the current script is located
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Copy the pre-commit hook to the .git/hooks directory
# List of hooks to copy
hooks=("pre-commit" "post-commit")

# Loop through each hook and copy it to the .git/hooks directory
for hook in "${hooks[@]}"; do
  cp "$DIR/hooks/$hook" "$DIR/../.git/hooks/$hook"
  chmod +x "$DIR/../.git/hooks/$hook"
done

# Update the submodules
git submodule update --init --recursive
git submodule update --recursive --remote