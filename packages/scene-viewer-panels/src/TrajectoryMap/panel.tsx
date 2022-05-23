import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import useGlobalVariables from "@foxglove/studio-base/hooks/useGlobalVariables";
import styled from "styled-components";
import { PinLocations } from "../SceneSelector/content";
import { useRosLib } from "../hooks/roslibHooks";
import { MapPanel as Map } from "./content";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

const params = new URLSearchParams(window.location.search);
const websocketUrl = params.get("ds.url") ?? "ws://localhost:9090";

function MapPanel() {
  const { currentPosition, trajectory } = useRosLib({
    websocketUrl,
    topicNames: [
      "/sensing/gnss/ublox/nav_sat_fix",
      "/scene_viewer/vehicle_trajectory",
    ],
  });

  const { globalVariables } = useGlobalVariables();
  const pinLocations = (globalVariables.pinLocations ?? []) as PinLocations;

  return (
    <Container>
      <PanelToolbar floating />
      <Map
        centerPosition={[35.1505536926114, 136.96585423505437]}
        currentPosition={[currentPosition.latitude, currentPosition.longitude]}
        markers={pinLocations.map((item) => {
          return {
            longitude: item.longitude,
            latitude: item.latitude,
            popupText: item.description,
            text: item.label,
          };
        })}
        polylines={[
          {
            positions: trajectory.map((item) => {
              return [item.latitude, item.longitude];
            }),
          },
        ]}
      />
    </Container>
  );
}

MapPanel.panelType = "TrajectoryMapPanel";
MapPanel.defaultConfig = {};

const Wrapped = Panel(MapPanel);

export { Wrapped as TrajectoryMap };
