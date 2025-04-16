FROM dimensionalos/unitree-ros

COPY docker /app/docker/

# Setup supervisor configuration
COPY docker/unitree/ros_dimos/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Copy entrypoint script
COPY docker/unitree/ros_dimos/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY dimos /app/dimos/
COPY tests /app/tests/

# Change working directory to /app for proper relative pathing
WORKDIR /app

# Create output directories for supervisord and ROS
RUN mkdir -p /app/assets/output/
RUN mkdir -p /app/assets/output/ros

# TODO: Cleanup multiple working directories and seprate the dockerfiles for each service.

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
