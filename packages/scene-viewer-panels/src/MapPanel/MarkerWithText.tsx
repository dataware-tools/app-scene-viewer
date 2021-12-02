import { divIcon } from "leaflet";
import React from "react";
import ReactDOMServer from "react-dom/server";
import { Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";

import { MarkerIcon, MarkerIconProps } from "../components/MarkerIcon";

export type MarkerWithTextProps = {
  longitude: number;
  latitude: number;
  popupText?: string;
} & MarkerIconProps;

export const MarkerWithText = ({
  longitude,
  latitude,
  popupText,
  text,
}: MarkerWithTextProps) => {
  const icon = divIcon({
    html: `
      <div style="
        position: relative;
        top: calc(-50% - 3px);
        left: calc(-50% - 3px);
      ">
        ${ReactDOMServer.renderToString(<MarkerIcon text={text} />)}
      </div>
    `,
  });

  return (
    <Marker position={[latitude, longitude]} icon={icon}>
      {popupText && <Popup>{popupText}</Popup>}
    </Marker>
  );
};
