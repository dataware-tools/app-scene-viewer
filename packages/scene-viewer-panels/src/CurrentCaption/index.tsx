import { css } from "@emotion/css";
import KeyboardArrowDownIcon from "@material-ui/icons/KeyboardArrowDown";
import KeyboardArrowUpIcon from "@material-ui/icons/KeyboardArrowUp";
import React from "react";
import { color } from "../color";
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

export const CurrentCaptionPresentation = ({
  captions: propsCaptions,
  currentSceneIndex,
  onChangeScene,
}: CurrentCaptionPresentationProps) => {
  const captions = [...propsCaptions];

  const determinePlacement = (itemIndex: number) => {
    const halfwayIndex = Math.ceil(captions.length / 2);
    const itemHeight = 50;

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

  const CircleIconButton = ({
    children,
    size,
    ...delegated
  }: { size: string } & React.DetailedHTMLProps<
    React.ButtonHTMLAttributes<HTMLButtonElement>,
    HTMLButtonElement
  >) => (
    <button
      className={css`
        align-items: center;
        background-color: ${color.gray(1)};
        border-radius: 100%;
        display: flex;
        height: ${size};
        justify-content: center;
        width: ${size};
      `}
      {...delegated}
    >
      {children}
    </button>
  );

  return (
    <>
      <div
        className={css`
          align-items: center;
          background-color: ${color.gray(1)};
          display: flex;
          flex-direction: row;
          height: 100%;
          overflow: auto;
          width: 100%;
        `}
      >
        <div
          className={css`
            display: flex;
            flex-direction: column;
            flex-grow: 1;
            height: 150px;
            overflow-y: hidden;
            position: relative;
          `}
        >
          {captions.map(({ timestamp, caption }, index) => {
            const isVisible =
              currentSceneIndex - 1 <= index && index <= currentSceneIndex + 1;
            const isDisplayed =
              currentSceneIndex - 2 <= index && index <= currentSceneIndex + 2;

            return (
              <div
                key={timestamp}
                className={css`
                  align-items: center;
                  bottom: 0;
                  color: ${index === currentSceneIndex
                    ? "white"
                    : color.gray(1)};
                  display: flex;
                  display: ${isDisplayed ? null : "none"};
                  flex-direction: row;
                  justify-content: center;
                  position: absolute;
                  top: 0;
                  transform: translateY(${determinePlacement(index)}px);
                  transition: transform 0.4s ease, opacity 0.4s ease;
                  visibility: ${isVisible ? "visible" : "hidden"};
                  width: 100%;
                `}
              >
                <span
                  className={css`
                    flex-shrink: 0;
                  `}
                >
                  {timestamp}
                </span>
                <span
                  className={css`
                    flex-shrink: 0;
                    width: 10px;
                  `}
                />
                <div
                  className={css`
                    overflow: hidden;
                    text-overflow: ${index !== currentSceneIndex && "ellipsis"};
                    white-space: ${index !== currentSceneIndex && "nowrap"};
                    word-break: ${index === currentSceneIndex && "break-all"};
                  `}
                >
                  {caption}
                </div>
              </div>
            );
          })}
        </div>
        <Spacer
          size={2}
          horizontal
          className={css`
            flex-shrink: 0;
          `}
        />
        <div
          className={css`
            align-items: center;
            display: flex;
            flex-direction: column;
            flex-shrink: 0;
            justify-content: center;
            overflow: hidden;
            padding: 5px 0;
          `}
        >
          <span>前のシーン</span>
          <Spacer size={1} vertical />
          <CircleIconButton
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex - 1])
            }
            disabled={currentSceneIndex < 1}
            size="30px"
          >
            <KeyboardArrowUpIcon fontSize="large" />
          </CircleIconButton>
          <Spacer vertical size={8} />
          <CircleIconButton
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex + 1])
            }
            disabled={currentSceneIndex >= captions.length - 1}
            size="30px"
          >
            <KeyboardArrowDownIcon fontSize="large" />
          </CircleIconButton>
          <Spacer vertical size={1} />
          <span>次のシーン</span>
        </div>
        <Spacer horizontal size={2} />
      </div>
    </>
  );
};

export const CurrentCaption = ({
  captions,
  currentTimestamp,
  onChangeScene,
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
      onChangeScene={onChangeScene}
      currentTimestamp={currentTimestamp}
    />
  );
};
