FROM dimensionalos/base-ros

RUN mkdir -p /app/dimos

COPY requirements.txt /app/
COPY base-requirements.txt /app/

WORKDIR /app

RUN pip install --no-cache-dir -r base-requirements.txt
RUN pip install --no-cache-dir -r requirements.txt
ENV PYTHONPATH=/app:${PYTHONPATH}
