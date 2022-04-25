import { css } from "@emotion/css";

export type MarkerIconProps = {
  color?: string;
  fontColor?: string;
  size?: number;
  text?: string;
};

export const MarkerIcon = ({
  color,
  fontColor,
  size,
  text,
}: MarkerIconProps) => {
  return (
    <div
      className={css`
        background-color: ${color};
        border-radius: 50%;
        color: ${fontColor};
        height: ${size}px;
        line-height: ${size}px;
        text-align: center;
        width: ${size}px;
      `}
    >
      {text}
    </div>
  );
};

MarkerIcon.defaultProps = {
  color: "#3388FE",
  fontColor: "#FFFFFF",
  size: 30,
  text: "",
};
