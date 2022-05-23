import SearchIcon from "@mui/icons-material/Search";
import { Index } from "flexsearch";
import { FormEvent, useEffect, useRef, useState } from "react";
import Highlighter from "react-highlight-words";
import { color } from "../color";
import { MarkerIcon } from "../components/MarkerIcon";
import { Spacer } from "../components/Spacer";
import Box from "@mui/material/Box";

type Caption = {
  timestamp: number;
  caption: string;
  location?: { latitude: number; longitude: number; altitude: number };
};
type CaptionWithLabel = {
  label?: string;
} & Caption;
export type PinLocations = {
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
    <Box
      sx={{
        alignItems: "center",
        display: "flex",
        flexDirection: "column",
        fontSize: "1.2rem",
        height: "100%",
        padding: "10px",
        width: "100%",
      }}
    >
      <Box
        component="form"
        onSubmit={(e: FormEvent) => {
          e.preventDefault();
          onSearch(inputRef.current?.value || "");
        }}
        sx={{
          display: "flex",
          flexDirection: "row",
          flexShrink: "0",
          position: "relative",
        }}
      >
        <SearchIcon
          fontSize="large"
          sx={{ left: "12px", position: "absolute", top: "4px" }}
        />
        <Box
          component="input"
          ref={inputRef}
          sx={{
            backgroundColor: color.gray(1),
            fontSize: "1.3rem",
            paddingLeft: "40px",
            "&:focus": {
              backgroundColor: color.gray(2),
            },
          }}
        />
        <Spacer size={2} horizontal />
        <Box
          component="button"
          onClick={() => onSearch(inputRef.current?.value || "")}
          sx={{
            backgroundColor: color.gray(2),
            fontSize: "1.3rem",
            fontWeight: "bold",
            padding: "0 20px",
            "&:hover": {
              backgroundColor: color.gray(3),
            },
          }}
        >
          Search
        </Box>
      </Box>
      <Spacer size={2} vertical />
      <Box
        sx={{
          flexShrink: "1",
          height: "100%",
          overflow: "auto",
          padding: "5px 10px",
          width: "100%",
        }}
      >
        <Box component="ul" sx={{ display: "inline-block" }}>
          {captionWithLabels.map(({ timestamp, caption, label }) => (
            <Box
              component="li"
              key={timestamp}
              sx={{
                alignItems: "center",
                cursor: "pointer",
                display: "flex",
                flexDirection: "row",
                padding: "5px",
                "&:hover": {
                  backgroundColor: color.gray(1),
                },
              }}
              onClick={() => onSelectScene(timestamp)}
            >
              <Box
                component="span"
                sx={{
                  alignItems: "center",
                  height: "20px",
                  justifyContent: "center",
                  width: "20px",
                }}
              >
                {label && <MarkerIcon size={20} text={label} />}
              </Box>
              <Spacer size={2} horizontal />
              <Box component="span" sx={{ fontWeight: "bold" }}>
                {timestamp}
              </Box>
              <Spacer size={2} horizontal />
              <Box component="span" sx={{ flexGrow: 1, whiteSpace: "nowrap" }}>
                <Highlighter
                  textToHighlight={caption}
                  searchWords={highlightedTexts}
                  highlightStyle={{
                    backgroundColor: "white",
                    fontWeight: "bold",
                    color: "black",
                  }}
                />
              </Box>
            </Box>
          ))}
        </Box>
      </Box>
    </Box>
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
