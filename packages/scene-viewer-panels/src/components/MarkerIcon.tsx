import { Box } from "@mui/system";

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
    <Box
      sx={{
        backgroundColor: color,
        borderRadius: "50%",
        color: fontColor,
        height: `${size}px`,
        lineHeight: `${size}px`,
        textAlign: "center",
        width: `${size}px`,
      }}
    >
      {text}
    </Box>
  );
};

MarkerIcon.defaultProps = {
  color: "#3388FE",
  fontColor: "#FFFFFF",
  size: 30,
  text: "",
};
