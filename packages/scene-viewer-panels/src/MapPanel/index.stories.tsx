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
  centerPosition: [35.1505536926114, 136.96585423505437],
};

export const MapWithMarkers = Template.bind({});
MapWithMarkers.args = {
  height: "500px",
  width: "500px",
  centerPosition: [35.1505536926114, 136.96585423505437],
  markers: [
    {
      longitude: 136.964871,
      latitude: 35.144697,
      popupText: "Yagoto Nisseki Station",
    },
    {
      longitude: 136.966588,
      latitude: 35.15447,
      popupText: "Nagoya University",
    },
  ],
};
