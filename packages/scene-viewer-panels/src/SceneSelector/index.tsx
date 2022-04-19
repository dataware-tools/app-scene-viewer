import { css } from "@emotion/css";
import SearchIcon from "@material-ui/icons/Search";
import { Index } from "flexsearch";
import React, { useEffect, useRef, useState } from "react";
import Highlighter from "react-highlight-words";
import { color } from "../color";
import { MarkerIcon } from "../components/MarkerIcon";
import { Spacer } from "../components/Spacer";

type Caption = {
  timestamp: number;
  caption: string;
  location?: { latitude: number; longitude: number; altitude: number };
};
type CaptionWithLabel = {
  label?: string;
} & Caption;
type PinLocations = {
  longitude: number;
  latitude: number;
  altitude: number;
  description?: string;
  timestamp?: number;
  label?: string;
}[];

export type SceneSelectorPresentationProps = {
  onSearch: (searchText: string) => void;
  highlightedTexts: string[];
  captionWithLabels: CaptionWithLabel[];
} & Omit<SceneSelectorProps, "setPinLocations" | "captions">;
export type SceneSelectorProps = {
  onSelectScene: (timestamp: number) => Promise<void> | void;
  captions: Caption[];
  setPinLocations: (pinLocation: PinLocations) => void | Promise<void>;
};

export const SceneSelectorPresentation = ({
  captionWithLabels,
  onSearch,
  highlightedTexts,
  onSelectScene,
}: SceneSelectorPresentationProps) => {
  const inputRef = useRef<HTMLInputElement>(null);
  return (
    <div
      className={css`
        align-items: center;
        background-color: ${color.gray(0)};
        display: flex;
        flex-direction: column;
        font-size: 1.2rem;
        height: 100%;
        padding: 10px;
        width: 100%;
      `}
    >
      <form
        onSubmit={(e) => {
          e.preventDefault();
          onSearch(inputRef.current?.value || "");
        }}
        className={css`
          display: flex;
          flex-direction: row;
          flex-shrink: 0;
          position: relative;
        `}
      >
        <SearchIcon
          fontSize="large"
          className={css`
            left: 12px;
            position: absolute;
            top: 4px;
          `}
        />
        <input
          ref={inputRef}
          className={css`
            background-color: ${color.gray(1)};
            font-size: 1.3rem;
            padding-left: 40px;
            &:focus {
              background-color: ${color.gray(2)};
            }
          `}
        />
        <Spacer size={2} horizontal />
        <button
          onClick={() => onSearch(inputRef.current?.value || "")}
          className={css`
            background-color: ${color.gray(2)};
            font-size: 1.3rem;
            font-weight: bold;
            padding: 0 20px;
            &:hover {
              background-color: ${color.gray(3)};
            }
          `}
        >
          Search
        </button>
      </form>
      <Spacer
        size={2}
        vertical
        className={css`
          flex-shrink: 0;
        `}
      />
      <div
        className={css`
          flex-shrink: 1;
          height: 100%;
          overflow: auto;
          padding: 5px 10px;
          width: 100%;
        `}
      >
        <ul
          className={css`
            display: inline-block;
          `}
        >
          {captionWithLabels.map(({ timestamp, caption, label }) => (
            <li
              key={timestamp}
              className={css`
                align-items: center;
                cursor: pointer;
                display: flex;
                flex-direction: row;
                padding: 5px;
                &:hover {
                  background-color: ${color.gray(1)};
                }
              `}
              onClick={() => onSelectScene(timestamp)}
            >
              <span
                className={css`
                  align-items: center;
                  height: 20px;
                  justify-content: center;
                  width: 20px;
                `}
              >
                {label && <MarkerIcon size={20} text={label} />}
              </span>
              <Spacer
                size={2}
                horizontal
                className={css`
                  flex-shrink: 0;
                `}
              />
              <span
                className={css`
                  font-weight: bold;
                `}
              >
                {timestamp}
              </span>
              <Spacer
                size={2}
                horizontal
                className={css`
                  flex-shrink: 0;
                `}
              />
              <span
                className={css`
                  flex-grow: 1;
                  white-space: nowrap;
                `}
              >
                <Highlighter
                  textToHighlight={caption}
                  searchWords={highlightedTexts}
                  highlightStyle={{
                    backgroundColor: "white",
                    fontWeight: "bold",
                    color: "black",
                  }}
                />
              </span>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
};

export const SceneSelector = ({
  captions,
  setPinLocations,
  ...delegated
}: SceneSelectorProps): JSX.Element => {
  const [highlightedTexts, setHighlightedTexts] = useState([""]);
  const [captionWithLabels, setCaptionWithLabels] = useState<
    CaptionWithLabel[]
  >([]);
  const flexSearchRef = useRef(
    new Index({
      tokenize: "full",
    })
  );

  const addLabelToCaptions = (captions: Caption[]) => {
    let numNonLabeledCaption = 0;
    const captionWithLabels = captions.map((caption, index) => {
      const label = caption.location
        ? (index + 1 - numNonLabeledCaption).toString()
        : undefined;
      if (!caption.location) {
        numNonLabeledCaption = numNonLabeledCaption + 1;
      }
      return { ...caption, label };
    });
    return captionWithLabels;
  };

  const convCaptionWithLabelsToPinLocations = (
    captionWithLabels: CaptionWithLabel[]
  ) => {
    const pinLocations = captionWithLabels
      .filter(({ label }) => label)
      .map(({ timestamp, caption, location, label }) => ({
        label: label,
        longitude: location?.longitude as number,
        latitude: location?.latitude as number,
        altitude: location?.altitude as number,
        description: caption,
        timestamp: timestamp,
      }));
    return pinLocations;
  };

  const setCaptions = (captions: Caption[]) => {
    const captionWithLabels = addLabelToCaptions(captions);
    setCaptionWithLabels(captionWithLabels);
    setPinLocations(convCaptionWithLabelsToPinLocations(captionWithLabels));
  };

  useEffect(() => {
    captions.forEach(({ caption }, index) =>
      flexSearchRef.current.add(index, caption)
    );

    setCaptions(captions);
  }, [captions]);
  // TODO: Need to include setCaption to deps↑ but that will lead to an infinite loop of calling setCaptions

  const onSearch: SceneSelectorPresentationProps["onSearch"] = (searchText) => {
    const fixedSearchText = searchText.replace("　", " ");
    setHighlightedTexts(fixedSearchText.split(" "));

    if (!fixedSearchText) {
      setCaptions(captions);
      return;
    }

    const searchResults = new Set(
      flexSearchRef.current.search(fixedSearchText)
    );
    const newCaptions = captions.filter((_, index) => searchResults.has(index));
    setCaptions(newCaptions);
  };

  return (
    <SceneSelectorPresentation
      captionWithLabels={captionWithLabels}
      highlightedTexts={highlightedTexts}
      onSearch={onSearch}
      {...delegated}
    />
  );
};
