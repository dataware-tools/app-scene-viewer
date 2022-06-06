import {
  MessagePipelineContext,
  useMessagePipeline,
} from "@foxglove/studio-base/components/MessagePipeline";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import styled from "styled-components";
import { useRosLib } from "../hooks/roslibHooks";
import { CurrentCaption } from "./content";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

const selectUrlState = (ctx: MessagePipelineContext) =>
  ctx.playerState.urlState;
function CurrentCaptionPanel(): JSX.Element {
  const playerUrlState = useMessagePipeline(selectUrlState);
  const { captions, seekToTimestamp, currentTime } = useRosLib({
    websocketUrl: playerUrlState?.parameters?.url ?? "ws://localhost:9090",
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
