FROM python:3.10.12

# Unitree Specific
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

# Change working directory to /app for proper relative pathing
WORKDIR /app

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

CMD [ "python", "-m", "tests.test_agent_alibaba" ]
