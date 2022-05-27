import { useTheme } from "@mui/material";
import { Box } from "@mui/system";

export type SpacerPresentationProps = {
  width: string;
  height: string;
};
export type SpacerProps = {
  size: number | string;
  vertical?: boolean;
  horizontal?: boolean;
};

export const SpacerPresentation = ({
  width,
  height,
}: SpacerPresentationProps): JSX.Element => {
  return (
    <Box
      component="span"
      sx={{
        height,
        width,
        display: "block",
        minHeight: height,
        minWidth: width,
      }}
    />
  );
};

export const Spacer = ({
  size = 1,
  horizontal,
  vertical,
  ...delegated
}: SpacerProps): JSX.Element => {
  const theme = useTheme();
  const fixedSize = typeof size === "number" ? theme.spacing(size) : size;
  const width = horizontal ? fixedSize : "0px";
  const height = vertical ? fixedSize : "0px";

  return <SpacerPresentation width={width} height={height} {...delegated} />;
};
