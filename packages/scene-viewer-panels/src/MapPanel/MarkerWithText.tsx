import { divIcon } from "leaflet";
import React from "react";
import { Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";

export type MarkerType = {
  longitude: number;
  latitude: number;
  popupText?: string;
  color?: string;
  size?: number;
};

export type MarkerWithTextProps = {
  text?: string;
} & MarkerType;

export const MarkerWithText = ({
  longitude,
  latitude,
  popupText,
  color,
  size,
  text,
}: MarkerWithTextProps) => {
  const icon = divIcon({
    html: `
      <div style="
        width: ${size}px;
        height: ${size}px;
        position: relative;
        top: calc(-50% - 3px);
        left: calc(-50% - 3px);
        background-color: ${color};
        line-height: ${size}px;
        text-align: center;
        border-radius: 50%;
      ">${text}</div>
    `,
  });

  return (
    <Marker position={[latitude, longitude]} icon={icon}>
      {popupText && <Popup>{popupText}</Popup>}
    </Marker>
  );
};

MarkerWithText.defaultProps = {
  color: "#3388FE",
  size: 30,
};
