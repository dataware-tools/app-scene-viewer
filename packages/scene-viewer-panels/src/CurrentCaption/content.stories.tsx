import type { Story } from "@storybook/react";
import { useState } from "react";
import { useRosLib } from "../hooks/roslibHooks";
import {
  CurrentCaption,
  CurrentCaptionProps,
  TimestampCaption,
} from "./content";

export default {
  component: CurrentCaption,
  title: "CurrentCaption",
};

const defaultCaptions = [
  { timestamp: 100000, caption: "1111111111111" },
  { timestamp: 100100, caption: "2222222222222" },
  { timestamp: 100200, caption: "3333333333333" },
  {
    timestamp: 100300,
    caption:
      "444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444",
  },
  {
    timestamp: 100400,
    caption: `55555555555555 55555555555 55555555555555 555555555555 555555555555 555555555555 555555555555 555555555555 555555555555 555555555555 555555555555`,
  },
  { timestamp: 100500, caption: "6666666666666" },
  { timestamp: 100600, caption: "7777777777777" },
  { timestamp: 100700, caption: "8888888888888" },
  { timestamp: 100800, caption: "9999999999999" },
  { timestamp: 100900, caption: "0000000000000" },
];
const Template: Story<
  CurrentCaptionProps & { height: string; width: string }
> = ({ height, width, ...args }) => (
  <div style={{ height, overflow: "auto", width }}>
    <CurrentCaption {...args} />
  </div>
);

export const Controlled = () => {
  const [currentTimestamp, setCurrentTimestamp] = useState(0);
  console.log(currentTimestamp);
  return (
    <CurrentCaption
      onChangeScene={({ timestamp }) => setCurrentTimestamp(timestamp)}
      currentTimestamp={currentTimestamp}
      captions={defaultCaptions}
    />
  );
};

export const WithRos = () => {
  const { captions, currentTime, seekToTimestamp } = useRosLib({
    topicNames: ["/scene_viewer/scene_captions", "/clock"],
  });
  return (
    <CurrentCaption
      onChangeScene={({ timestamp }) => seekToTimestamp(timestamp)}
      currentTimestamp={currentTime}
      captions={captions as TimestampCaption[]}
    />
  );
};
// Skip on VRT, because this story relies backends
WithRos.story = { parameters: { loki: { skip: true } } };

export const InLargeContainer = Template.bind({});
InLargeContainer.args = {
  width: "100%",
  height: "100%",
  captions: defaultCaptions,
  currentTimestamp: 100350,
  onChangeScene: ({ timestamp, caption }) => {
    window.confirm(`Seek to ${timestamp} (caption is "${caption}")`);
  },
};

export const InSmallContainer = Template.bind({});
InSmallContainer.args = {
  width: "200px",
  height: "100px",
  captions: defaultCaptions,
  currentTimestamp: 100350,
  onChangeScene: ({ timestamp, caption }) => {
    window.confirm(`Seek to ${timestamp} (caption is "${caption}")`);
  },
};
