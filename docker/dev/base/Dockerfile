FROM dimensionalos/unitree-agents-interface

RUN apt-get update && apt-get install -y \
    fish \
    git 

COPY motd /etc/motd

RUN echo "cat /etc/motd" >> /root/.bashrc
