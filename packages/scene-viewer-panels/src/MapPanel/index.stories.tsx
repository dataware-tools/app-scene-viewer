import { css } from "@emotion/css";
import type { Story } from "@storybook/react";
import React from "react";
import { MapPanel, MapPanelProps } from "./index";

export default {
  component: MapPanel,
  title: "MapPanel",
};

const Template: Story<MapPanelProps & { height: string; width: string }> = ({
  height,
  width,
  ...args
}) => (
  <div
    className={css`
      height: ${height};
      width: ${width};
    `}
  >
    <MapPanel {...args} />
  </div>
);

export const Default = Template.bind({});
Default.args = {
  height: "500px",
  width: "500px",
  centerPosition: [35.1537602, 136.9641892],
  markers: [
    {
      longitude: 136.9641892,
      latitude: 35.1537602,
      popupText: "Nagoya University",
    },
  ],
};
