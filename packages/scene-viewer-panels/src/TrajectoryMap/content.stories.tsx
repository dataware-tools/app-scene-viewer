import type { Story } from "@storybook/react";
import { useRosLib } from "../hooks/roslibHooks";
import { MapPanel, MapPanelProps } from "./content";

export default {
  component: MapPanel,
  title: "MapPanel",
};

const Template: Story<MapPanelProps & { height: string; width: string }> = ({
  height = "500px",
  width = "500px",
  ...args
}) => (
  <div style={{ height, width }}>
    <MapPanel {...args} />
  </div>
);

const defaultCenterPosition: [number, number] = [
  35.1505536926114, 136.96585423505437,
];
const defaultCurrentPosition: [number, number] = [
  35.14819909438752, 136.9651522986249,
];
const defaultMarkers = [
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
];

export const Default = Template.bind({});
Default.args = {
  centerPosition: defaultCenterPosition,
};
// Skip on VRT, because this story is too flaky
Default.story = { parameters: { loki: { skip: true } } };

export const MapWithMarkers = Template.bind({});
MapWithMarkers.args = {
  centerPosition: defaultCenterPosition,
  markers: defaultMarkers,
};
// Skip on VRT, because this story is too flaky
MapWithMarkers.story = { parameters: { loki: { skip: true } } };

export const MapWithAllAnnotations = Template.bind({});
MapWithAllAnnotations.args = {
  centerPosition: defaultCenterPosition,
  currentPosition: defaultCurrentPosition,
  markers: defaultMarkers,
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
// Skip on VRT, because this story is too flaky
MapWithAllAnnotations.story = { parameters: { loki: { skip: true } } };

export const WithRos = () => {
  const { currentPosition, trajectory, captionsWithLocation } = useRosLib({
    topicNames: [
      "/sensing/gnss/ublox/nav_sat_fix",
      "/scene_viewer/vehicle_trajectory",
      "/scene_viewer/scene_captions_with_locations",
    ],
  });
  return (
    <div style={{ height: "500px", width: "500px" }}>
      <MapPanel
        centerPosition={[35.1505536926114, 136.96585423505437]}
        currentPosition={[currentPosition.latitude, currentPosition.longitude]}
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
// Skip on VRT, because this story relies backends
WithRos.story = { parameters: { loki: { skip: true } } };
