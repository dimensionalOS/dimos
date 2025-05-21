FROM python:3

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    build-essential \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libpostproc-dev \ 
    gcc \
    make \
    portaudio19-dev \
    python3-pyaudio \
    python3-all-dev

WORKDIR /app

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

CMD [ "python", "-m", "dimos.robot.unitree.unitree_go2" ]
