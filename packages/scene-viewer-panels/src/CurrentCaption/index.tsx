import { css } from "@emotion/css";
import React from "react";

type TimestampCaption = { timestamp: number; caption: string };
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
  currentSceneIndex: propsCurrentSceneIndex,
  onChangeScene,
}: CurrentCaptionPresentationProps) => {
  const captions = [...propsCaptions];
  const dummyEl = {
    timestamp: null as unknown as number,
    caption: "",
  };
  captions.unshift(dummyEl, dummyEl);
  captions.push(dummyEl, dummyEl);

  const currentSceneIndex = propsCurrentSceneIndex + 2;
  return (
    <>
      <div
        className={css`
          align-items: center;
          display: flex;
          flex-direction: row;
          overflow: auto;
        `}
      >
        <div
          className={css`
            display: flex;
            flex-direction: column;
            flex-grow: 1;
            justify-content: center;
            overflow: auto;
          `}
        >
          <ul>
            {captions
              .map(({ timestamp, caption }, index) => {
                return (
                  <li
                    key={timestamp}
                    className={css`
                      align-items: center;
                      color: ${index === currentSceneIndex ? "white" : "gray"};
                      display: flex;
                      flex-direction: row;
                      padding: 5px 0;
                    `}
                  >
                    <span>{timestamp}</span>
                    <span
                      className={css`
                        flex-shrink: 0;
                        width: 10px;
                      `}
                    />
                    <span
                      className={css`
                        white-space: nowrap;
                      `}
                    >
                      {caption}
                    </span>
                  </li>
                );
              })
              .filter(
                (_, index) =>
                  currentSceneIndex - 1 <= index &&
                  index <= currentSceneIndex + 1
              )}
          </ul>
        </div>
        <span
          className={css`
            flex-shrink: 0;
            width: 10px;
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
          <span
            className={css`
              height: 5px;
            `}
          />
          <button
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex - 1])
            }
            disabled={currentSceneIndex < 1 + 2}
            className={css`
              border-radius: 100%;
            `}
          >
            ↑
          </button>
          <span
            className={css`
              height: 10px;
            `}
          />
          <button
            onClick={async () =>
              await onChangeScene(captions[currentSceneIndex + 1])
            }
            disabled={currentSceneIndex >= captions.length - (1 + 2)}
            className={css`
              border-radius: 100%;
            `}
          >
            ↓
          </button>
          <span
            className={css`
              height: 5px;
            `}
          />
          <span>次のシーン</span>
        </div>
        <span
          className={css`
            width: 10px;
          `}
        />
      </div>
    </>
  );
};

export const CurrentCaption = ({
  captions,
  currentTimestamp,
  onChangeScene,
}: CurrentCaptionProps): JSX.Element => {
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
