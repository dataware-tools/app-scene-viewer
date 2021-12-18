import { css } from "@emotion/css";
import GpsFixedIcon from "@material-ui/icons/GpsFixed";
import Leaflet from "leaflet";
import React, { useCallback, useEffect, useState } from "react";
import ReactDOMServer from "react-dom/server";
import { MapContainer, TileLayer, Pane, Polyline } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { CurrentLocationMarker } from "./CurrentLocationMarker";
import { MarkerWithText, MarkerWithTextProps } from "./MarkerWithText";

Leaflet.Icon.Default.imagePath =
  "//cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/";
const mapInitialZoom = 15;

type PolylineType = {
  positions: Leaflet.LatLngExpression[];
};

export type MapPanelPresentationProps = MapPanelProps & {
  setMap: (map: Leaflet.Map) => void;
};

export type MapPanelProps = {
  centerPosition: Leaflet.LatLngExpression;
  currentPosition?: Leaflet.LatLngExpression;
  markers?: MarkerWithTextProps[];
  polylines?: PolylineType[];
};

export const MapPanelPresentation = ({
  centerPosition,
  currentPosition,
  markers,
  polylines,
  setMap,
}: MapPanelPresentationProps) => {
  return (
    <MapContainer
      center={centerPosition}
      zoom={mapInitialZoom}
      scrollWheelZoom={false}
      whenCreated={setMap}
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
          markers.map((marker: MarkerWithTextProps, i: number) => {
            return (
              <MarkerWithText
                key={i}
                longitude={marker.longitude}
                latitude={marker.latitude}
                popupText={marker.popupText}
                size={30}
                text={marker.text}
              />
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
  const [map, setMap] = useState<Leaflet.Map | undefined>(undefined);
  const [trackCurrentPosition, setTrackCurrentPosition] = useState(false);

  // Move center position of map
  useEffect(() => {
    if (map && currentPosition && trackCurrentPosition) {
      map.setView(currentPosition, mapInitialZoom);
    }
  }, [map, currentPosition, trackCurrentPosition]);

  // Add onDragStart callback to map component
  const onDragStart = useCallback(() => {
    setTrackCurrentPosition(false);
  }, []);
  useEffect(() => {
    if (!map) return;
    map.on("dragstart", onDragStart);
    return () => {
      map.off("dragstart", onDragStart);
    };
  }, [map, onDragStart]);

  // Add button element to enable tracking position on map
  useEffect(() => {
    if (!map) return;
    const MapHelp = Leaflet.Control.extend({
      onAdd: () => {
        const helpDiv = Leaflet.DomUtil.create("div", "leaflet-bar");
        helpDiv.innerHTML = `
          ${ReactDOMServer.renderToString(
            <a
              role="button"
              className={css`
                cursor: pointer;
                font-size: 22px;
              `}
            >
              <GpsFixedIcon />
            </a>
          )}
        `;
        helpDiv.addEventListener("click", () => {
          setTrackCurrentPosition(true);
        });
        return helpDiv;
      },
    });
    const trackCurrentPositionButton = new MapHelp({ position: "bottomleft" });
    trackCurrentPositionButton.addTo(map);
  }, [map, setTrackCurrentPosition]);

  return (
    <MapPanelPresentation
      centerPosition={centerPosition}
      currentPosition={currentPosition}
      markers={markers}
      polylines={polylines}
      setMap={setMap}
    />
  );
};
