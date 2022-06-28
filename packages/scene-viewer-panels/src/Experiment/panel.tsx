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
  useEffect(() => {
    if (
      !linkedGlobalVariables.some(
        (linkedGlobalVariable) => linkedGlobalVariable.name === "experiment"
      )
    ) {
      setLinkedGlobalVariables([
        ...linkedGlobalVariables,
        {
          markerKeyPath: ["position", "pose"],
          name: "experiment",
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
          width: "100%",
          height: "100%",
          justifyContent: "center",
          alignItems: "center",
          fontSize: "50px",
        }}
      >
        {JSON.stringify(globalVariables.experiment)}
      </div>
    </Container>
  );
}

ExperimentPanel.panelType = "CurrentCaptionPanel";
ExperimentPanel.defaultConfig = {};

const Wrapped = Panel(ExperimentPanel);

export { Wrapped as Experiment };
