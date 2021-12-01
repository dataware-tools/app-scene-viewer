import { css } from "@emotion/css";
import Leaflet from "leaflet";
import React from "react";
import {
  MapContainer,
  TileLayer,
  Marker,
  Pane,
  Polyline,
  Popup,
} from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { CurrentLocationMarker } from "./CurrentLocationMarker";

Leaflet.Icon.Default.imagePath =
  "//cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/";

type MarkerType = {
  longitude: number;
  latitude: number;
  popupText?: string;
};

type PolylineType = {
  positions: Leaflet.LatLngExpression[];
};

export type MapPanelPresentationProps = MapPanelProps;

export type MapPanelProps = {
  centerPosition: Leaflet.LatLngExpression;
  currentPosition?: Leaflet.LatLngExpression;
  markers?: MarkerType[];
  polylines?: PolylineType[];
};

export const MapPanelPresentation = ({
  centerPosition,
  currentPosition,
  markers,
  polylines,
}: MapPanelPresentationProps) => {
  return (
    <MapContainer
      center={centerPosition}
      zoom={15}
      scrollWheelZoom={false}
      className={css`
        height: 100%;
        width: 100%;
      `}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />

      <Pane name="polylines" style={{ zIndex: 800 }}>
        {polylines &&
          polylines.map((polyline: PolylineType, i: number) => {
            return <Polyline positions={polyline.positions} key={i} />;
          })}
      </Pane>

      <Pane name="markers" style={{ zIndex: 900 }}>
        {markers &&
          markers.map((marker: MarkerType, i: number) => {
            return (
              <Marker position={[marker.latitude, marker.longitude]} key={i}>
                {marker.popupText && <Popup>{marker.popupText}</Popup>}
              </Marker>
            );
          })}
      </Pane>

      <Pane name="current-position" style={{ zIndex: 1000 }}>
        {currentPosition && (
          <CurrentLocationMarker position={currentPosition} />
        )}
      </Pane>
    </MapContainer>
  );
};

export const MapPanel = ({
  centerPosition,
  currentPosition,
  markers,
  polylines,
}: MapPanelProps): JSX.Element => {
  return (
    <MapPanelPresentation
      centerPosition={centerPosition}
      currentPosition={currentPosition}
      markers={markers}
      polylines={polylines}
    />
  );
};
