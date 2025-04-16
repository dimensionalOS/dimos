FROM dimensionalos/unitree-ros

# Install basic requirements
RUN apt-get update && apt-get install -y \
    supervisor \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Create dimos directory structure
RUN mkdir -p /app/dimos /app/docker

COPY requirements.txt /app/

WORKDIR /app

# Install dimos requirements
RUN pip install --no-cache-dir -r requirements.txt

# Set PYTHONPATH permanently
ENV PYTHONPATH=/app:${PYTHONPATH}

COPY docker /app/docker/

# Setup supervisor configuration
COPY docker/unitree/agents/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Copy entrypoint script
COPY docker/unitree/agents/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Copy dimos and tests
COPY dimos /app/dimos/
COPY tests /app/tests
COPY dimos/__init__.py /app/__init__.py

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Create output directories for supervisord and ROS
RUN mkdir -p /app/assets/output/
RUN mkdir -p /app/assets/output/ros

# TODO: Cleanup multiple working directories and seprate the dockerfiles for each service.

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
