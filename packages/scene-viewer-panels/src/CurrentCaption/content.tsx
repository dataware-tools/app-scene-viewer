import KeyboardArrowDownIcon from "@mui/icons-material/KeyboardArrowDown";
import KeyboardArrowUpIcon from "@mui/icons-material/KeyboardArrowUp";
import Box from "@mui/material/Box";
import MuiIconButton, {
  IconButtonProps as MuiIconButtonProps,
} from "@mui/material/IconButton";
import Paper from "@mui/material/Paper";
import { Spacer } from "../components/Spacer";

export type TimestampCaption = { timestamp: number; caption: string };
export type CurrentCaptionPresentationProps = {
  currentSceneIndex: number;
} & CurrentCaptionProps;
export type CurrentCaptionProps = {
  captions: TimestampCaption[];
  currentTimestamp: number;
  onChangeScene: (newValue: TimestampCaption) => void | Promise<void>;
};

const IconButton = ({ children, ...delegated }: MuiIconButtonProps) => (
  <Paper
    elevation={4}
    sx={{
      borderRadius: "100%",
    }}
  >
    <MuiIconButton color="inherit" size="small" {...delegated}>
      {children}
    </MuiIconButton>
  </Paper>
);

export const CurrentCaptionPresentation = ({
  captions: propsCaptions,
  currentSceneIndex,
  onChangeScene,
}: CurrentCaptionPresentationProps) => {
  const captions = [...propsCaptions];

  const determinePlacement = (itemIndex: number) => {
    const halfwayIndex = Math.ceil(captions.length / 2);
    const itemHeight = 70;

    if (currentSceneIndex === itemIndex) return 0;
    if (itemIndex >= halfwayIndex) {
      if (currentSceneIndex > itemIndex - halfwayIndex) {
        return (itemIndex - currentSceneIndex) * itemHeight;
      } else {
        return -(captions.length + currentSceneIndex - itemIndex) * itemHeight;
      }
    }

    if (itemIndex > currentSceneIndex) {
      return (itemIndex - currentSceneIndex) * itemHeight;
    }

    if (itemIndex < currentSceneIndex) {
      if (currentSceneIndex - itemIndex >= halfwayIndex) {
        return (captions.length - (currentSceneIndex - itemIndex)) * itemHeight;
      }
      return -(currentSceneIndex - itemIndex) * itemHeight;
    }
    return null;
  };

  return (
    <>
      <Box
        sx={{
          alignItems: "center",
          display: "flex",
          flexDirection: "row",
          height: "100%",
          overflow: "auto",
          fontSize: "1.1rem",
          width: "100%",
        }}
      >
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            flexGrow: 1,
            height: "200px",
            overflowY: "hidden",
            position: "relative",
          }}
        >
          {captions.map(({ timestamp, caption }, index) => {
            const isVisible =
              currentSceneIndex - 1 <= index && index <= currentSceneIndex + 1;
            const isDisplayed =
              currentSceneIndex - 2 <= index && index <= currentSceneIndex + 2;

            return (
              <Box
                key={timestamp}
                sx={{
                  alignItems: "center",
                  bottom: 0,
                  color: (theme) =>
                    index === currentSceneIndex
                      ? theme.palette.text.primary
                      : theme.palette.grey[500],
                  display: isDisplayed ? "flex" : "none",
                  flexDirection: "row",
                  justifyContent: "center",
                  position: "absolute",
                  top: 0,
                  fontSize: currentSceneIndex === index ? "1.3rem" : null,
                  transform: `translateY(${determinePlacement(index)}px)`,
                  transition: "transform 0.4s ease, opacity 0.4s ease",
                  visibility: isVisible ? "visible" : "hidden",
                  width: "100%",
                }}
              >
                <Box
                  sx={{
                    overflow: "hidden",
                    paddingLeft: "10px",
                    paddingRight: "10px",
                    textOverflow:
                      index !== currentSceneIndex ? "ellipsis" : null,
                    whiteSpace: index !== currentSceneIndex ? "nowrap" : null,
                    wordBreak: index === currentSceneIndex ? "break-all" : null,
                  }}
                >
                  {caption}
                </Box>
              </Box>
            );
          })}
        </Box>
        <Spacer size={2} horizontal />
        <Box
          sx={{
            alignItems: "center",
            display: "flex",
            flexDirection: "column",
            flexShrink: "0",
            justifyContent: "center",
            overflow: "hidden",
            padding: "5px 0",
          }}
        >
          <span>前のシーン</span>
          <Spacer size={1} vertical />
          <IconButton
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex - 1])
            }
            disabled={currentSceneIndex < 1}
          >
            <KeyboardArrowUpIcon />
          </IconButton>
          <Spacer vertical size={8} />
          <IconButton
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex + 1])
            }
            disabled={currentSceneIndex >= captions.length - 1}
          >
            <KeyboardArrowDownIcon />
          </IconButton>
          <Spacer vertical size={1} />
          <span>次のシーン</span>
        </Box>
        <Spacer horizontal size={2} />
      </Box>
    </>
  );
};

export const CurrentCaption = ({
  captions,
  currentTimestamp,
  ...delegated
}: CurrentCaptionProps): JSX.Element => {
  if (captions.length <= 0) {
    return <div>No captions available</div>;
  }
  const currentSceneIndex =
    captions[captions.length - 1].timestamp <= currentTimestamp
      ? captions.length - 1
      : captions.findIndex(
          ({ timestamp }, index) =>
            timestamp <= currentTimestamp &&
            currentTimestamp < captions[index + 1].timestamp
        );

  return (
    <CurrentCaptionPresentation
      captions={captions}
      currentSceneIndex={currentSceneIndex}
      currentTimestamp={currentTimestamp}
      {...delegated}
    />
  );
};
