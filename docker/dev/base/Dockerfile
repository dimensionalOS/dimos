FROM dimensionalos/unitree-agents-interface

RUN apt-get update && apt-get install -y \
    fish \
    git 

ARG GIT_COMMIT=unknown
ARG GIT_BRANCH=unknown

# Copy files and add version to motd
COPY motd /etc/motd
COPY bash.sh /root/.bash.sh
RUN echo "  v_${GIT_BRANCH}:${GIT_COMMIT} | $(date)\n" >> /etc/motd

RUN echo "cat /etc/motd" >> /root/.bashrc
RUN echo "source /root/.bash.sh" >> /root/.bashrc

