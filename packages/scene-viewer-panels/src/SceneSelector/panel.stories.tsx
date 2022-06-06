import PanelSetup from "@foxglove/studio-base/stories/PanelSetup";
import { SceneSelector } from "./panel";

export default {
  title: "panels/SceneSelector",
  component: SceneSelector,
};

export function Default(): JSX.Element {
  return (
    <PanelSetup>
      <SceneSelector />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
Default.story = { parameters: { loki: { skip: true } } };

export function NarrowLayout(): JSX.Element {
  return (
    <PanelSetup style={{ width: 400 }}>
      <SceneSelector />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
NarrowLayout.story = { parameters: { loki: { skip: true } } };
