import { css } from "@emotion/css";
import Leaflet from "leaflet";
import React from "react";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";

Leaflet.Icon.Default.imagePath =
  "//cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/";

type MarkerType = {
  longitude: number;
  latitude: number;
  popupText?: string;
};

export type MapPanelPresentationProps = MapPanelProps;

export type MapPanelProps = {
  centerPosition: Leaflet.LatLngExpression;
  markers?: MarkerType[];
};

export const MapPanelPresentation = ({
  centerPosition,
  markers,
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

      {markers &&
        markers.map((marker: MarkerType, i: number) => {
          return (
            <Marker position={[marker.latitude, marker.longitude]} key={i}>
              {marker.popupText && <Popup>{marker.popupText}</Popup>}
            </Marker>
          );
        })}
    </MapContainer>
  );
};

export const MapPanel = ({
  centerPosition,
  markers,
}: MapPanelProps): JSX.Element => {
  return (
    <MapPanelPresentation centerPosition={centerPosition} markers={markers} />
  );
};
