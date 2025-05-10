FROM dimensionalos/unitree-agents-interface

ARG GIT_COMMIT=unknown
ARG GIT_BRANCH=unknown

RUN apt-get update -y

RUN apt-get install -y \
    fish \
    git \
    nano \
    vim \
    ccze \
    tmux \
    htop \
    python-is-python3 \
    iputils-ping \
    wget \
    fzf \
    autojump \
    exa

RUN mkdir /app/.shell && mkdir -p ./.local/share/autojump/
RUN touch /app/.shell/.autojump.txt; ln -s /app/.shell/.autojump.txt ./.local/share/autojump/autojump.txt

# Copy files and add version to motd
COPY motd /etc/motd
COPY bash.sh /root/.bash.sh
COPY tmux.conf /root/.tmux.conf

RUN echo "  v_${GIT_BRANCH}:${GIT_COMMIT} | $(date)" >> /etc/motd
RUN echo "echo -e '\033[34m$(cat /etc/motd)\033[0m\n'" >> /root/.bashrc
RUN git config --global --add safe.directory /app

RUN echo "source /root/.bash.sh" >> /root/.bashrc
