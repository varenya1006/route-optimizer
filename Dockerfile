FROM python:3.11-slim

# Install system dependencies for the geospatial stack (OSMnx, GeoPandas, Shapely, Rtree)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    libgeos-dev \
    libproj-dev \
    gdal-bin \
    libgdal-dev \
    libspatialindex-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Application code
COPY . .

# Persistent cache directory for OSM graph
RUN mkdir -p /app/cache

ENV FLASK_APP=app.py
ENV PYTHONUNBUFFERED=1
ENV GRAPH_CACHE_DIR=/app/cache

EXPOSE 5000

CMD ["python", "app.py"]