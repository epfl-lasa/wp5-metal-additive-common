FROM universalrobots/ursim_cb3

# Install the URCap
COPY ./programs/externalcontrol-1.0.5.urcap /urcaps/externalcontrol-1.0.5.jar

SHELL ["/bin/bash", "-c"]
RUN apt update --fix-missing && apt upgrade -y && apt clean

RUN apt install -y \
    default-jdk

# Install the URCap
RUN mkdir -p /ursim/programs

ENTRYPOINT [ "./entrypoint.sh" ]
