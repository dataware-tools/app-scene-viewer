import CancelIcon from "@mui/icons-material/Cancel";
import SaveIcon from "@mui/icons-material/Save";
import { Typography, useTheme } from "@mui/material";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import React, { useState, useEffect } from "react";
import { PopoverIconButton } from "./PopoverIconButton";

type CommentInputProps<T extends "toggleEditable" | "alwaysEditable"> = {
  mode?: T;
  editing?: T extends "toggleEditable" ? boolean : never;
  onSave: (comment: string) => void | Promise<void>;
  onCancel?: (comment: string) => void | Promise<void>;
  comment?: string;
  clearOnSave?: boolean;
};

export const CommentInput = <T extends "toggleEditable" | "alwaysEditable">({
  mode,
  onSave,
  onCancel,
  comment: initComment,
  editing,
}: CommentInputProps<T>) => {
  const [comment, setComment] = useState(initComment || "");
  const [prevComment, setPrevComment] = useState(initComment || "");
  const [open, setOpen] = useState(false);

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setComment(event.target.value);
  };

  useEffect(() => {
    setComment(initComment || "");
    setPrevComment(initComment || "");
  }, [initComment]);

  const theme = useTheme();

  return mode === "toggleEditable" && !editing ? (
    <Typography
      onDoubleClick={() => setOpen((prev) => !prev)}
      sx={{
        whiteSpace: "pre-wrap",
        wordBreak: "break-all",
        width: "100%",
        ...(open
          ? {}
          : {
              overflow: "hidden",
              display: "-webkit-box",
              "-webkit-box-orient": "vertical",
              "-webkit-line-clamp": "3",
            }),
      }}
    >
      {comment}
    </Typography>
  ) : (
    <Stack
      component="form"
      direction="row"
      spacing={1}
      onSubmit={() => {
        onSave(comment);
      }}
      width="100%"
    >
      <TextField
        value={comment}
        onChange={handleChange}
        variant="standard"
        multiline
        fullWidth
        inputProps={{
          style: {
            color: theme.palette.text.primary,
          },
        }}
        InputProps={{
          sx: {
            borderBottom: `1px solid ${theme.palette.text.primary}`,
            "&:before": {
              borderBottom: `1px solid ${theme.palette.text.primary}`,
            },
          },
        }}
      />
      <PopoverIconButton
        iconButtonProps={{
          children: <SaveIcon />,
          onClick: () => {
            onSave(comment);
            switch (mode) {
              case "toggleEditable":
                setPrevComment(comment);
                break;
              case "alwaysEditable":
                setComment("");
                break;
            }
          },
        }}
        popoverProps={{ children: "save" }}
      />
      <PopoverIconButton
        iconButtonProps={{
          children: <CancelIcon />,
          onClick: () => {
            onCancel && onCancel(comment);
            switch (mode) {
              case "toggleEditable":
                setComment(prevComment);
                break;
              case "alwaysEditable":
                setComment("");
                break;
            }
          },
        }}
        popoverProps={{ children: "cancel" }}
      />
    </Stack>
  );
};
