import {
  MessagePipelineContext,
  useMessagePipeline,
} from "@foxglove/studio-base/components/MessagePipeline";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import useGlobalVariables from "@foxglove/studio-base/hooks/useGlobalVariables";
import React from "react";
import styled from "styled-components";
import { useRosLib } from "../hooks/roslibHooks";
import { PinLocations, SceneSelector } from "./content";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

const selectUrlState = (ctx: MessagePipelineContext) =>
  ctx.playerState.urlState;
function SceneSelectorPanel() {
  const playerUrlState = useMessagePipeline(selectUrlState);
  const { captionsWithLocation, seekToTimestamp } = useRosLib({
    websocketUrl: playerUrlState?.parameters?.url ?? "ws://localhost:9090",
    topicNames: ["/scene_viewer/scene_captions_with_locations"],
  });
  const { setGlobalVariables } = useGlobalVariables();

  const setPinLocations = React.useCallback(
    (pinLocations: PinLocations) => {
      setGlobalVariables({ pinLocations });
    },
    [setGlobalVariables]
  );
  const captions = React.useMemo(
    () =>
      captionsWithLocation.map((item) => {
        return {
          timestamp: item.timestamp,
          caption: item.caption,
          location: {
            altitude: item.altitude,
            latitude: item.latitude,
            longitude: item.longitude,
          },
        };
      }),
    [captionsWithLocation]
  );

  return (
    <Container>
      <PanelToolbar floating />
      <SceneSelector
        captions={captions}
        setPinLocations={setPinLocations}
        onSelectScene={(timestamp) => {
          seekToTimestamp(timestamp);
        }}
      />
    </Container>
  );
}

SceneSelectorPanel.panelType = "SceneSelectorPanel";
SceneSelectorPanel.defaultConfig = {};

const Wrapped = Panel(SceneSelectorPanel);

export { Wrapped as SceneSelector };
