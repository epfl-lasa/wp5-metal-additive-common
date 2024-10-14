#!/bin/bash
# -*- coding: utf-8 -*-

# Script to help calling every setup script
# Author: Louis Munier - <lmunier@protonmail.com>
# Date: 2024-10-09
bash scripts/setup_git.sh

docker compose build
docker compose up -d
