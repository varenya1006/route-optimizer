FROM python:3.11-slim

WORKDIR /app

# System deps for osmnx / shapely
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgeos-dev libspatialindex-dev gdal-bin libgdal-dev && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 5000

# gunicorn for production; long timeout because OSM graph takes ~20s to load
CMD ["python", "-c", \
     "from app import app, load_osm_graph; load_osm_graph(); app.run(host='0.0.0.0', port=5000)"]
