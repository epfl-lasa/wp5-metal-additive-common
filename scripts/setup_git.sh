#!/bin/bash
# -*- coding: utf-8 -*-

# Script to help setup git hooks
# Author: Louis Munier - <lmunier@protonmail.com>
# Date: 2024-10-09

# Get the directory where the current script is located
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Copy the pre-commit hook to the .git/hooks directory
cp "$DIR/hooks/pre-commit" "$DIR/../.git/hooks/pre-commit"
chmod +x .git/hooks/pre-commit

# Update the submodules
git submodule update --init --recursive
git submodule update --recursive --remote