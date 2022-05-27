import PanelSetup from "@foxglove/studio-base/stories/PanelSetup";
import { SceneSelector } from "./panel";

export default {
  title: "panels/SceneSelector",
  component: SceneSelector,
};

export function Example(): JSX.Element {
  return (
    <PanelSetup>
      <SceneSelector />
    </PanelSetup>
  );
}

export function NarrowLayout(): JSX.Element {
  return (
    <PanelSetup>
      <div style={{ width: 400 }}>
        <SceneSelector />
      </div>
    </PanelSetup>
  );
}
