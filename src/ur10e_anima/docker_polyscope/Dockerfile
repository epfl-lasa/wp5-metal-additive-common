FROM universalrobots/ursim_e-series

SHELL ["/bin/bash", "-c"]
RUN apt update --fix-missing && apt upgrade -y && apt clean

RUN apt install -y \
    default-jdk

# Install the URCap
RUN mkdir -p /ursim/programs && \
    mkdir -p /ursim/urcaps

ENTRYPOINT [ "./entrypoint.sh" ]
