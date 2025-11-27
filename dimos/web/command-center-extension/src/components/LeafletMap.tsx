import * as React from "react";
import { MapContainer, TileLayer, Marker, Popup, useMapEvents } from "react-leaflet";
import L, { LatLngExpression } from "leaflet";
import { LatLon } from "../types";

// Fix for default marker icons in react-leaflet
// Using CDN URLs since webpack can't handle the image imports directly
const DefaultIcon = L.icon({
  iconUrl: "https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png",
  shadowUrl: "https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png",
  iconSize: [25, 41],
  iconAnchor: [12, 41],
});

L.Marker.prototype.options.icon = DefaultIcon;

// Component to handle map click events
function MapClickHandler({ onMapClick }: { onMapClick: (lat: number, lng: number) => void }) {
  useMapEvents({
    click: (e) => {
      onMapClick(e.latlng.lat, e.latlng.lng);
    },
  });
  return null;
}

interface LeafletMapProps {
  gpsLocation: LatLon | null;
  gpsTravelGoalPoints: LatLon[] | null;
  onGpsGoal: (goal: LatLon) => void;
}

const LeafletMap: React.FC<LeafletMapProps> = ({ gpsLocation, gpsTravelGoalPoints, onGpsGoal }) => {
  if (!gpsLocation) {
    return (
      <div style={{
        width: "100%",
        height: "100%",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        fontSize: "18px",
        color: "#666"
      }}>
        GPS location not received yet.
      </div>
    );
  }

  const center: LatLngExpression = [gpsLocation.lat, gpsLocation.lon];

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <style>{leafletCss}</style>
      <MapContainer
        center={center}
        zoom={14}
        style={{ width: "100%", height: "100%" }}
      >
        <TileLayer
          attribution=''
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <MapClickHandler onMapClick={(lat: number, lng: number) => {
          onGpsGoal({ lat, lon: lng });
        }} />
        <Marker position={center}>
          <Popup>Current GPS Location</Popup>
        </Marker>
        {gpsTravelGoalPoints !== null && (
          gpsTravelGoalPoints.map(p => (
            <Marker key={`${p.lat}_${p.lon}}`} position={[p.lat, p.lon]}></Marker>
          ))
        )}
      </MapContainer>
    </div>
  );
};

const leafletCss = `
.leaflet-control-container {
  display: none;
}
.leaflet-container {
  width: 100%;
  height: 100%;
  position: relative;
}
.leaflet-pane,
.leaflet-tile,
.leaflet-marker-icon,
.leaflet-marker-shadow,
.leaflet-tile-container,
.leaflet-pane > svg,
.leaflet-pane > canvas,
.leaflet-zoom-box,
.leaflet-image-layer,
.leaflet-layer {
  position: absolute;
  left: 0;
  top: 0;
}
.leaflet-container {
  overflow: hidden;
  -webkit-tap-highlight-color: transparent;
  background: #ddd;
  outline: 0;
  font: 12px/1.5 "Helvetica Neue", Arial, Helvetica, sans-serif;
}
.leaflet-tile {
  filter: inherit;
  visibility: hidden;
}
.leaflet-tile-loaded {
  visibility: inherit;
}
.leaflet-zoom-box {
  width: 0;
  height: 0;
  -moz-box-sizing: border-box;
  box-sizing: border-box;
  z-index: 800;
}
.leaflet-control {
  position: relative;
  z-index: 800;
  pointer-events: visiblePainted;
  pointer-events: auto;
}
.leaflet-top,
.leaflet-bottom {
  position: absolute;
  z-index: 1000;
  pointer-events: none;
}
.leaflet-top {
  top: 0;
}
.leaflet-right {
  right: 0;
}
.leaflet-bottom {
  bottom: 0;
}
.leaflet-left {
  left: 0;
}
`;

export default LeafletMap;
