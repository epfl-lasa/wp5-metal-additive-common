FROM universalrobots/ursim_e-series

SHELL ["/bin/bash", "-c"]
RUN apt update --fix-missing && apt upgrade -y && apt clean

# Install the URCap
RUN mkdir -p /ursim/programs && \
    mkdir -p /ursim/urcaps

COPY externalcontrol-1.0.5.urcap /ursim/urcaps/externalcontrol-1.0.5.jar

ENTRYPOINT [ "./entrypoint.sh" ]
