import CancelIcon from "@mui/icons-material/Cancel";
import SaveIcon from "@mui/icons-material/Save";
import { useTheme } from "@mui/material";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import { useState } from "react";
import { PopoverIconButton } from "./PopoverIconButton";

export type InputProps = {
  onSave?: (text: string, clearText: () => void) => void;
  onCancel?: (text: string, clearText: () => void) => void;
  initComment?: string;
};

export const CommentInput = ({ onSave, onCancel, initComment }: InputProps) => {
  const [comment, setComment] = useState(initComment || "");
  const theme = useTheme();

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setComment(event.target.value);
  };
  const clearText = () => setComment("");
  return (
    <Stack
      component="form"
      direction="row"
      spacing={1}
      onSubmit={() => {
        onSave && onSave(comment, clearText);
      }}
      width="100%"
      alignItems="center"
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
          onClick: () => onSave && onSave(comment, clearText),
        }}
        popoverProps={{ children: "save" }}
      />
      <PopoverIconButton
        iconButtonProps={{
          children: <CancelIcon />,
          onClick: () => onCancel && onCancel(comment, clearText),
        }}
        popoverProps={{ children: "cancel" }}
      />
    </Stack>
  );
};
