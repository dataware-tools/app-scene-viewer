import PanelSetup from "@foxglove/studio-base/stories/PanelSetup";
import { CurrentCaption } from "./panel";

const fixture = {
  topics: [
    { name: "/scene_viewer/scene_captions", datatype: "std_msgs/String" },
    { name: "/baz/text", datatype: "baz/text" },
    { name: "/baz/array", datatype: "baz/array" },
    { name: "/baz/array/obj", datatype: "baz/array/obj" },
  ],
  frame: {
    "/scene_viewer/scene_captions": [
      {
        topic: "/scene_viewer/scene_captions",
        receiveTime: { sec: 122, nsec: 456789011 },
        message: {
          some_array: ["a", "b", "c"],
          some_deleted_key: "GONE",
          some_id_example_2: { some_id: 0 },
        },
        sizeInBytes: 0,
      },
      {
        topic: "/scene_viewer/scene_captions",
        receiveTime: { sec: 123, nsec: 456789012 },
        message: {
          some_array: ["a", "b", "c", "d", "e", "f"],
          some_id_example_2: { some_id: 123 },
        },
        sizeInBytes: 0,
      },
    ],
  },
  datatypes: new Map(
    Object.entries({
      "std_msgs/String": { definitions: [{ name: "value", type: "string" }] },
    })
  ),
};

export default {
  title: "panels/CurrentCaption",
  component: CurrentCaption,
};

export function Example(): JSX.Element {
  return (
    <PanelSetup fixture={fixture}>
      <CurrentCaption />
    </PanelSetup>
  );
}

export function NarrowLayout(): JSX.Element {
  return (
    <PanelSetup fixture={fixture}>
      <div style={{ width: 400 }}>
        <CurrentCaption />
      </div>
    </PanelSetup>
  );
}
