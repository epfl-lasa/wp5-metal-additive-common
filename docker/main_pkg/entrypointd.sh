#!/bin/sh

# https://github.com/sudo-bmitch/docker-base/blob/main/bin/entrypointd.sh
# Copyright: Brandon Mitchell
# License: MIT

set -e
# Handle a kill signal before the final "exec" command runs
trap "{ exit 0; }" TERM INT

# strip off "/bin/sh -c" args from a string CMD
if [ $# -gt 1 ] && [ "$1" = "/bin/sh" ] && [ "$2" = "-c" ]; then
  shift 2
  eval "set -- $1"
fi

if [ -f /.volume-cache/volume-list.already-run ]; then
  rm /.volume-cache/volume-list.already-run
fi

for ep in /etc/entrypoint.d/*; do
  ext="${ep##*.}"
  if [ "${ext}" = "env" ] && [ -f "${ep}" ]; then
    # source files ending in ".env"
    echo "Sourcing: ${ep} $@"
    set -a && . "${ep}" "$@" && set +a
  elif [ "${ext}" = "sh" ] && [ -x "${ep}" ]; then
    # run scripts ending in ".sh"
    echo "Running: ${ep} $@"
    "${ep}" "$@"
  fi
done

# inject certificates
if [ -d /etc/certs.d ]; then
  add-certs
fi

# load any cached volumes
if [ -f /.volume-cache/volume-list -a ! -f /.volume-cache/volume-list.already-run ]; then
  load-volume -a
fi

# Default to the prior entrypoint if defined
if [ -n "$ORIG_ENTRYPOINT" ]; then
  set -- "$ORIG_ENTRYPOINT" "$@"
fi

# run a shell if there is no command passed
if [ $# = 0 ]; then
  if [ -x /bin/bash ]; then
    set -- /bin/bash
  else
    set -- /bin/sh
  fi
fi

# include tini if requested
if [ -n "${USE_INIT}" ]; then
  set -- tini -- "$@"
fi

# ------------------------------------------------------------------------------
USER=robetarme_user

# fix stdout/stderr permissions to allow non-root user
chown --dereference $USER "/proc/$$/fd/1" "/proc/$$/fd/2" || :

# In case the host user's uid is not equal to the container's user's uid (1000)
# the bind mount `/home/$USER/catkin_ws/src` (see docker-compose.yml:volumes)
# is owned by the host's user. If we transfer ownership to the container's user
# then we simply shuffle the problem. Give others rwX rights to mitigate the
# bulk of this discrepancy.
chmod -R o+rwX /home/$USER/catkin_ws/src
# ------------------------------------------------------------------------------
# Drop from root to robetarme_user at the very last minute as per best practice.
# Then execute /bin/bash.
#
# However, if you want your container to execute something upon starting it,
# then comment out the following two lines and jump to the next instruction
# below.
set -- gosu $USER "$@"
exec "$@"

# Launch your node as a normal user upon entry to the container.
# The template command is `sudo su - ${USER} -c COMMAND`.
#sudo su - ${USER} -c "source /opt/ros/noetic/setup.bash; \
                    #export DISPLAY=":0"; \
                    #roslaunch robetarme_bt_launchers avanti_my_pkg.launch"
# ------------------------------------------------------------------------------