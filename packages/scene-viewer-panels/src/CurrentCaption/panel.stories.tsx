import PanelSetup from "@foxglove/studio-base/stories/PanelSetup";
import { CurrentCaption } from "./panel";

export default {
  title: "panels/CurrentCaption",
  component: CurrentCaption,
};

export function Default(): JSX.Element {
  return (
    <PanelSetup>
      <CurrentCaption />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
Default.story = { parameters: { loki: { skip: true } } };

export function NarrowLayout(): JSX.Element {
  return (
    <PanelSetup style={{ width: 400 }}>
      <CurrentCaption />
    </PanelSetup>
  );
}
// Skip on VRT, because this story relies backends
NarrowLayout.story = { parameters: { loki: { skip: true } } };
