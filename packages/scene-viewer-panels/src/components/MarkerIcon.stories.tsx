import type { Story } from "@storybook/react";
import React from "react";
import { MarkerIcon, MarkerIconProps } from "./MarkerIcon";

export default {
  component: MarkerIcon,
  title: "Components/MarkerIcon",
};

const Template: Story<MarkerIconProps> = ({ ...args }) => (
  <MarkerIcon {...args} />
);

export const Default = Template.bind({});
Default.args = {
  color: "#3388FE",
  fontColor: "#FFFFFF",
  size: 30,
  text: "999",
};
