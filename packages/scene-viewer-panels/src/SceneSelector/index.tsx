import { css } from "@emotion/css";
import SearchIcon from "@material-ui/icons/Search";
import { Index } from "flexsearch";
import React, { useEffect, useRef, useState } from "react";
import Highlighter from "react-highlight-words";

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
  const white = "hsl(0, 0%, 94%)";
  const gray = "hsl(240, 2%, 53%)";
  const lightgray = "hsl(240, 2%, 70%)";
  const red = "hsl(3, 82%, 54%)";
  return (
    <div
      className={css`
        align-items: center;
        background-color: hsl(240deg 1% 41%);
        border-radius: 5px;
        color: white;
        display: flex;
        flex-direction: column;
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
            color: ${gray};
            left: 12px;
            position: absolute;
            top: 4px;
          `}
        />
        <input
          ref={inputRef}
          className={css`
            background-color: ${white};
            color: ${gray};
            font-size: 1.25rem;
            padding-left: 40px;
            &:focus {
              background-color: ${white};
              color: ${gray};
            }
          `}
        />
        <span
          className={css`
            width: 10px;
          `}
        />
        <button
          onClick={() => onSearch(inputRef.current?.value || "")}
          className={css`
            background-color: ${white};
            color: black;
            font-size: 1.25rem;
            font-weight: bold;
            padding: 0 20px;
            &:hover {
              background-color: ${white} !important;
              color: ${gray} !important;
            }
          `}
        >
          Search
        </button>
      </form>
      <span
        className={css`
          flex-shrink: 0;
          height: 10px;
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
                  background-color: ${lightgray};
                }
              `}
              onClick={() => onSelectScene(timestamp)}
            >
              <span
                className={css`
                  align-items: center;
                  background-color: ${label ? red : undefined};
                  border: ${label ? `1px solid ${white}` : undefined};
                  border-radius: 100%;
                  color: ${label ? white : undefined};
                  display: flex;
                  flex-shrink: 0;
                  font-weight: bold;
                  height: 20px;
                  justify-content: center;
                  width: 20px;
                `}
              >
                {label || ""}
              </span>
              <span
                className={css`
                  flex-shrink: 0;
                  width: 10px;
                `}
              />
              <span
                className={css`
                  font-weight: bold;
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
                    backgroundColor: "hsl(197, 39%, 58%)",
                    fontWeight: "bold",
                    color: "white",
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
