import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import styled from "styled-components";
import { useRosLib } from "../hooks/roslibHooks";
import { CurrentCaption } from "./content";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

const params = new URLSearchParams(window.location.search);
const websocketUrl = params.get("ds.url") ?? "ws://localhost:9090";

function CurrentCaptionPanel(): JSX.Element {
  const { captions, seekToTimestamp, currentTime } = useRosLib({
    websocketUrl,
    topicNames: ["/scene_viewer/scene_captions", "/clock"],
  });
  return (
    <Container>
      <PanelToolbar floating />
      <CurrentCaption
        onChangeScene={({ timestamp }) => seekToTimestamp(timestamp)}
        currentTimestamp={currentTime}
        captions={captions}
      />
    </Container>
  );
}

CurrentCaptionPanel.panelType = "CurrentCaptionPanel";
CurrentCaptionPanel.defaultConfig = {};

const Wrapped = Panel(CurrentCaptionPanel);

export { Wrapped as CurrentCaption };
