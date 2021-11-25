import { css } from "@emotion/css";
import type { Story } from "@storybook/react";
import React, { useState } from "react";
import { CurrentCaption, CurrentCaptionProps } from "./index";

export default {
  component: CurrentCaption,
  title: "CurrentCaption",
};

const Template: Story<CurrentCaptionProps & { height: string; width: string }> =
  ({ height, width, ...args }) => (
    <div
      className={css`
        height: ${height};
        overflow: auto;
        width: ${width};
      `}
    >
      <CurrentCaption {...args} />
    </div>
  );

export const Controlled = () => {
  const [currentTimestamp, setCurrentTimestamp] = useState(0);
  console.log(currentTimestamp);
  return (
    <div
      className={css`
        height: 150px;
        overflow: auto;
        width: 500px;
      `}
    >
      <CurrentCaption
        onChangeScene={({ timestamp }) => setCurrentTimestamp(timestamp)}
        currentTimestamp={currentTimestamp}
        captions={[
          { timestamp: 100000, caption: "1111111111111" },
          { timestamp: 100100, caption: "2222222222222" },
          { timestamp: 100200, caption: "3333333333333" },
          {
            timestamp: 100300,
            caption:
              "4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444",
          },
          {
            timestamp: 100400,
            caption: `55555555555555
      55555555555
      55555555555555
      555555555555`,
          },
          { timestamp: 100500, caption: "6666666666666" },
          { timestamp: 100600, caption: "7777777777777" },
          { timestamp: 100700, caption: "8888888888888" },
          { timestamp: 100800, caption: "9999999999999" },
          { timestamp: 100900, caption: "0000000000000" },
        ]}
      />
    </div>
  );
};
export const Default = Template.bind({});
Default.args = {
  height: "150px",
  width: "500px",
  captions: [
    { timestamp: 100000, caption: "1111111111111" },
    { timestamp: 100100, caption: "2222222222222" },
    { timestamp: 100200, caption: "3333333333333" },
    {
      timestamp: 100300,
      caption:
        "4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444",
    },
    {
      timestamp: 100400,
      caption: `55555555555555
      55555555555
      55555555555555
      555555555555`,
    },
    { timestamp: 100500, caption: "6666666666666" },
    { timestamp: 100600, caption: "7777777777777" },
    { timestamp: 100700, caption: "8888888888888" },
    { timestamp: 100800, caption: "9999999999999" },
    { timestamp: 100900, caption: "0000000000000" },
  ],
  currentTimestamp: 100345,
  onChangeScene: ({ timestamp, caption }) => {
    window.confirm(`Seek to ${timestamp} (caption is "${caption}")`);
  },
};

export const InLargeContainer = Template.bind({});

InLargeContainer.args = {
  width: "100%",
  height: "100%",
  captions: [
    { timestamp: 100000, caption: "1111111111111" },
    { timestamp: 100100, caption: "2222222222222" },
    { timestamp: 100200, caption: "3333333333333" },
    {
      timestamp: 100300,
      caption:
        "4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444",
    },
    {
      timestamp: 100400,
      caption: `55555555555555
      55555555555
      55555555555555
      555555555555`,
    },
    { timestamp: 100500, caption: "6666666666666" },
    { timestamp: 100600, caption: "7777777777777" },
    { timestamp: 100700, caption: "8888888888888" },
    { timestamp: 100800, caption: "9999999999999" },
    { timestamp: 100900, caption: "0000000000000" },
  ],
  currentTimestamp: 100050,
  onChangeScene: ({ timestamp, caption }) => {
    window.confirm(`Seek to ${timestamp} (caption is "${caption}")`);
  },
};

export const InSmallContainer = Template.bind({});

InSmallContainer.args = {
  width: "200px",
  height: "50px",
  captions: [
    { timestamp: 100000, caption: "1111111111111" },
    { timestamp: 100100, caption: "2222222222222" },
    { timestamp: 100200, caption: "3333333333333" },
    {
      timestamp: 100300,
      caption:
        "4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444",
    },
    {
      timestamp: 100400,
      caption: `55555555555555
      55555555555
      55555555555555
      555555555555`,
    },
    { timestamp: 100500, caption: "6666666666666" },
    { timestamp: 100600, caption: "7777777777777" },
    { timestamp: 100700, caption: "8888888888888" },
    { timestamp: 100800, caption: "9999999999999" },
    { timestamp: 100900, caption: "0000000000000" },
  ],
  currentTimestamp: 111111,
  onChangeScene: ({ timestamp, caption }) => {
    window.confirm(`Seek to ${timestamp} (caption is "${caption}")`);
  },
};
