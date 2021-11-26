import { css } from "@emotion/css";
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
  captions: Caption[];
  setPinLocations: (pinLocation: PinLocations) => void | Promise<void>;
};

export const SceneSelectorPresentation = ({
  captionWithLabels,
  onSearch,
  highlightedTexts,
}: SceneSelectorPresentationProps) => {
  const inputRef = useRef<HTMLInputElement>(null);
  return (
    <div
      className={css`
        align-items: center;
        display: flex;
        flex-direction: column;
        height: 100%;
        padding: 10px;
        width: 100%;
      `}
    >
      <div
        className={css`
          flex-shrink: 0;
        `}
      >
        <form
          onSubmit={(e) => {
            e.preventDefault();
            onSearch(inputRef.current?.value || "");
          }}
        >
          <input ref={inputRef} placeholder="search..." />
          <button onClick={() => onSearch(inputRef.current?.value || "")}>
            Search
          </button>
        </form>
      </div>
      <span
        className={css`
          flex-shrink: 0;
          height: 10px;
        `}
      />
      <div
        className={css`
          border: 3px solid gray;
          border-radius: 5px;
          flex-shrink: 1;
          height: 100%;
          overflow: auto;
          padding: 5px 10px;
          width: 100%;
        `}
      >
        <ul>
          {captionWithLabels.map(({ timestamp, caption, label }) => (
            <li
              key={timestamp}
              className={css`
                align-items: center;
                display: flex;
                flex-direction: row;
                padding: 5px 0;
              `}
            >
              <span
                className={css`
                  background-color: ${label ? "red" : undefined};
                  border-radius: 100%;
                  color: ${label ? "black" : undefined};
                  flex-shrink: 0;
                  text-align: center;
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
                <Highlighter
                  textToHighlight={caption}
                  searchWords={highlightedTexts}
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
  }, []);

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
    />
  );
};
