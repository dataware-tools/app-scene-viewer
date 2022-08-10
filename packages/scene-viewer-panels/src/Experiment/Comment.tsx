import DeleteIcon from "@mui/icons-material/Delete";
import EditIcon from "@mui/icons-material/Edit";
import RadioButtonCheckedIcon from "@mui/icons-material/RadioButtonChecked";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";
import { CommentInput } from "./CommentInput";
import { PopoverIconButton } from "./PopoverIconButton";
import { ClickedPointType } from "./type";

export type CommentType = {
  annotationId: number;
  text: string;
  target: { point: ClickedPointType["point"] };
  frameId: ClickedPointType["header"]["frame_id"];
};

export const Comment = ({
  comment,
  highlight,
  onStartEdit,
  onSaveEdit,
  onCancelEdit,
  onDelete,
  onSelect,
  editing,
}: {
  comment: CommentType;
  highlight?: boolean;
  onStartEdit: () => void;
  onSaveEdit: (comment: string) => void;
  onCancelEdit: () => void;
  onDelete: () => void;
  onSelect: () => void;
  editing?: boolean;
}) => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "row",
        alignItems: "center",
        justifyContent: "space-between",
        width: "100%",
        backgroundColor: highlight ? (theme) => theme.palette.info.main : null,
        borderRadius: "5px",
      }}
    >
      <Stack
        direction="row"
        spacing={2}
        sx={{ alignItems: "center", width: "100%" }}
      >
        <Box
          sx={{
            borderRadius: "100%",
            border: "solid",
            width: "2rem",
            height: "2rem",
            fontSize: "1rem",
            display: "grid",
            placeItems: "center",
            flexShrink: 0,
          }}
        >
          {comment.annotationId}
        </Box>
        <CommentInput
          comment={comment.text}
          onSave={onSaveEdit}
          onCancel={onCancelEdit}
          mode="toggleEditable"
          editing={editing}
        />
      </Stack>
      {editing ? null : (
        <Stack spacing={1} direction="row">
          <PopoverIconButton
            iconButtonProps={{
              children: <RadioButtonCheckedIcon />,
              onClick: onSelect,
            }}
            popoverProps={{ children: "select point" }}
          />
          <PopoverIconButton
            iconButtonProps={{
              children: <EditIcon />,
              onClick: onStartEdit,
            }}
            popoverProps={{ children: "edit" }}
          />
          <PopoverIconButton
            iconButtonProps={{
              color: "error",
              children: <DeleteIcon />,
              onClick: onDelete,
            }}
            popoverProps={{ children: "delete" }}
          />
        </Stack>
      )}
    </Box>
  );
};
