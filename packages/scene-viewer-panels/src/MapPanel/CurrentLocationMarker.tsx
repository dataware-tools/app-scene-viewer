import Leaflet from "leaflet";
import React from "react";
import { CircleMarker } from "react-leaflet";
import "leaflet/dist/leaflet.css";

export type CurrentLocationMarkerProps = {
  position: Leaflet.LatLngExpression;
};

export const CurrentLocationMarker = ({
  position,
}: CurrentLocationMarkerProps) => {
  const pathOptions = {
    color: "#E3EEFF",
    fillColor: "#3388FE",
    fillOpacity: 1,
    weight: 5,
  };
  return (
    <CircleMarker center={position} pathOptions={pathOptions} radius={10} />
  );
};
