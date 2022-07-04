import { useMessageDataItem } from "@foxglove/studio-base/components/MessagePathSyntax/useMessageDataItem";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import useGlobalVariables from "@foxglove/studio-base/hooks/useGlobalVariables";
import useLinkedGlobalVariables from "@foxglove/studio-base/panels/ThreeDimensionalViz/Interactions/useLinkedGlobalVariables";
import { useEffect } from "react";
import styled from "styled-components";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

function ExperimentPanel(): JSX.Element {
  const { linkedGlobalVariables, setLinkedGlobalVariables } =
    useLinkedGlobalVariables();
  const { globalVariables } = useGlobalVariables();
  const matchedMessages = useMessageDataItem("/tf");
  const tfRaw =
    // @ts-expect-error ignore error
    matchedMessages[matchedMessages.length - 1]?.queriedData[0]?.value
      .transforms;
  const tf = tfRaw?.map((el) => ({
    frame_id: el.child_frame_id,
    rotation: el.transform.rotation,
    translation: el.transform.translation,
  }));

  useEffect(() => {
    if (
      !linkedGlobalVariables.some(
        (linkedGlobalVariable) => linkedGlobalVariable.name === "position"
      )
    ) {
      setLinkedGlobalVariables([
        ...linkedGlobalVariables,
        {
          markerKeyPath: ["position", "pose"],
          name: "position",
          topic: "/markers/annotations",
        },
      ]);
    }
    if (
      !linkedGlobalVariables.some(
        (linkedGlobalVariable) => linkedGlobalVariable.name === "frame_id"
      )
    ) {
      setLinkedGlobalVariables([
        ...linkedGlobalVariables,
        {
          markerKeyPath: ["frame_id", "header"],
          name: "frame_id",
          topic: "/markers/annotations",
        },
      ]);
    }
  }, [linkedGlobalVariables]);

  return (
    <Container>
      <PanelToolbar floating />
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          fontSize: "1.5rem",
        }}
      >
        <span>position: {JSON.stringify(globalVariables.position)}</span>
        <span>header.frame_id: {globalVariables.frame_id}</span>
        <span>tf: </span>
        <div
          style={{
            paddingLeft: 20,
            display: "flex",
            flexDirection: "column",
            fontSize: "0.9rem",
          }}
        >
          {tf?.map(({ frame_id, rotation, translation }) => (
            <div
              key={frame_id}
              style={{
                display: "flex",
                flexDirection: "column",
              }}
            >
              <span>frame_id: {frame_id}</span>
              <span
                style={{
                  display: "flex",
                  flexDirection: "column",
                  paddingLeft: 20,
                }}
              >
                <span>rotation: {JSON.stringify(rotation)}</span>
                <span>translation: {JSON.stringify(translation)}</span>
              </span>
            </div>
          ))}
        </div>
      </div>
    </Container>
  );
}

ExperimentPanel.panelType = "CurrentCaptionPanel";
ExperimentPanel.defaultConfig = {};

const Wrapped = Panel(ExperimentPanel);

export { Wrapped as Experiment };
