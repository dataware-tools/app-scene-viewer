import { css, cx } from "@emotion/css";
import React from "react";

export type SpacerPresentationProps = {
  width: string;
  height: string;
} & React.DetailedHTMLProps<
  React.HTMLAttributes<HTMLSpanElement>,
  HTMLSpanElement
>;

export type SpacerProps = {
  size: number | string;
  vertical?: boolean;
  horizontal?: boolean;
} & React.DetailedHTMLProps<
  React.HTMLAttributes<HTMLSpanElement>,
  HTMLSpanElement
>;

export const SpacerPresentation = ({
  width,
  height,
  className: propsClassName,
  ...delegated
}: SpacerPresentationProps): JSX.Element => {
  return (
    <span
      className={cx(
        css`
          display: "block";
          height: ${height};
          min-height: ${height};
          min-width: ${width};
          width: ${width};
        `,
        propsClassName
      )}
      {...delegated}
    />
  );
};

export const Spacer = ({
  size = 1,
  horizontal,
  vertical,
  ...delegated
}: SpacerProps): JSX.Element => {
  const fixedSize = typeof size === "number" ? `${size * 5}px` : size;
  const width = horizontal ? fixedSize : "0px";
  const height = vertical ? fixedSize : "0px";

  return <SpacerPresentation width={width} height={height} {...delegated} />;
};
