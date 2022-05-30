import PanelSetup from "@foxglove/studio-base/stories/PanelSetup";
import { TrajectoryMap } from "./panel";
import { PinLocations } from "../SceneSelector/content";

export default {
  title: "panels/TrajectoryMap",
  component: TrajectoryMap,
};
const fixture: { globalVariables: { pinLocations: PinLocations } } = {
  globalVariables: {
    pinLocations: [
      {
        altitude: 1,
        longitude: 136.964871,
        latitude: 35.144697,
        description: "Yagoto Nisseki Station",
        label: "1",
      },
      {
        altitude: 1,
        longitude: 136.966588,
        latitude: 35.15447,
        description: "Nagoya University",
        label: "2",
      },
    ],
  },
};

export function Default(): JSX.Element {
  return (
    <PanelSetup fixture={fixture} style={{ width: "500px", height: "500px" }}>
      <TrajectoryMap />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
Default.story = { parameters: { loki: { skip: true } } };

export function NarrowLayout(): JSX.Element {
  return (
    <PanelSetup fixture={fixture} style={{ width: "200px", height: "200px" }}>
      <TrajectoryMap />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
NarrowLayout.story = { parameters: { loki: { skip: true } } };
