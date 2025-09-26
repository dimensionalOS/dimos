# OpenStreetMap (OSM)

This provides functionality to fetch and work with OpenStreetMap tiles, including coordinate conversions and location-based VLM queries.

## Getting a MapImage

```python
map_image = get_osm_map(LatLon(lat=..., lon=...), zoom_level=18, n_tiles=4)`
```

OSM tiles are 256x256 pixels so with 4 tiles you get a 1024x1024 map.

You can translate pixel coordinates on the map to GPS location and back.

```python
>>> map_image.pixel_to_latlon((300, 500))
LatLon(lat=43.58571248, lon=12.23423511)
>>> map_image.latlon_to_pixel(LatLon(lat=43.58571248, lon=12.23423511))
(300, 500)
```

## CurrentLocationMap

This class maintains an appropriate context map for your current location so you can VLM queries.

You have to update it with your current location and when you stray too far from the center it fetches a new map.

```python
curr_map = CurrentLocationMap(QwenVlModel())

# Set your latest position. 
curr_map.update_position(LatLon(lat=..., lon=...))

# If you want to get back a GPS position of a feature (Qwen gets your current position).
curr_map.query_for_one_position('Where is the closest farmacy?')
# Returns:
#     LatLon(lat=..., lon=...)

# If you also want to get back a description of the result.
curr_map.query_for_one_position_and_context('Where is the closest pharmacy?')
# Returns:
#     (LatLon(lat=..., lon=...), "Lloyd's Pharmacy on Main Street")
```
