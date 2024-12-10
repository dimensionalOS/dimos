FROM python:3

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx

WORKDIR /app

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

COPY ./dimos ./dimos

COPY ./demos ./demos

COPY ./tests ./tests

COPY ./dimos/__init__.py ./

# CMD [ "python", "-m", "dimos-env.tests.test_environment" ]

# CMD [ "python", "-m", "dimos-env.tests.test_openai_agent_v3" ]

CMD [ "python", "-m", "tests.test_agent" ]
