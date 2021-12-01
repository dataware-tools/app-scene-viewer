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

export const MapWithAllAnnotations = Template.bind({});
MapWithAllAnnotations.args = {
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
  polylines: [
    {
      positions: [
        [35.144697, 136.964871],
        [35.146017695754104, 136.9644168258311],
        [35.15154533435691, 136.96603391120726],
        [35.15319804374357, 136.96580931601613],
        [35.15447, 136.966588],
      ],
    },
  ],
};
