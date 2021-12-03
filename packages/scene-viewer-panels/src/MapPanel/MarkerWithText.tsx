import { divIcon } from "leaflet";
import React, { useMemo } from "react";
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
  size,
  text,
}: MarkerWithTextProps) => {
  const positionShift = useMemo(() => {
    return size ? size / 2 - 3 : 0;
  }, [size]);

  const icon = divIcon({
    html: `
      <div style="
        position: relative;
        top: -${positionShift}px;
        left: -${positionShift}px;
      ">
        ${ReactDOMServer.renderToString(<MarkerIcon text={text} size={size} />)}
      </div>
    `,
    popupAnchor: [0, -positionShift - 5],
  });

  return (
    <Marker position={[latitude, longitude]} icon={icon}>
      {popupText && <Popup>{popupText}</Popup>}
    </Marker>
  );
};

MarkerWithText.defaultProps = {
  size: 30,
};
