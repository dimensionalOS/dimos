FROM python:3

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    ffmpeg

WORKDIR /app

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

COPY ./dimos ./dimos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

CMD [ "python", "-m", "tests.test_standalone_rtsp_streamer" ]
