import { css } from "@emotion/css";
import type { Story } from "@storybook/react";
import React from "react";
import { useRosLib } from "../hooks/roslibHooks";
import { MapPanel, MapPanelProps } from "./index";

export default {
  component: MapPanel,
  title: "Panels/MapPanel",
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
      text: "1",
    },
    {
      longitude: 136.966588,
      latitude: 35.15447,
      popupText: "Nagoya University",
      text: "2",
    },
  ],
};

export const MapWithAllAnnotations = Template.bind({});
MapWithAllAnnotations.args = {
  height: "500px",
  width: "500px",
  centerPosition: [35.1505536926114, 136.96585423505437],
  currentPosition: [35.14819909438752, 136.9651522986249],
  markers: [
    {
      longitude: 136.964871,
      latitude: 35.144697,
      popupText: "Yagoto Nisseki Station",
      text: "1",
    },
    {
      longitude: 136.966588,
      latitude: 35.15447,
      popupText: "Nagoya University",
      text: "2",
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

export const WithRos = () => {
  const { trajectory, captionsWithLocation } = useRosLib({
    topicNames: [
      "/scene_viewer/vehicle_trajectory",
      "/scene_viewer/scene_captions_with_locations",
    ],
  });
  return (
    <div
      className={css`
        height: 500px;
        width: 500px;
      `}
    >
      <MapPanel
        centerPosition={[35.1505536926114, 136.96585423505437]}
        markers={captionsWithLocation.map((item) => {
          return {
            longitude: item.longitude,
            latitude: item.latitude,
            popupText: item.caption,
          };
        })}
        polylines={[
          {
            positions: trajectory.map((item) => {
              return [item.latitude, item.longitude];
            }),
          },
        ]}
      />
    </div>
  );
};
