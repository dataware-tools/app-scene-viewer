import { IconButton, IconButtonProps, useTheme } from "@mui/material";
import Popover, { PopoverProps } from "@mui/material/Popover";
import React, { useState } from "react";

type PopoverIconButtonProps = {
  iconButtonProps: IconButtonProps;
  popoverProps: Omit<PopoverProps, "open" | "anchorEl">;
};
export const PopoverIconButton = ({
  iconButtonProps,
  popoverProps,
}: PopoverIconButtonProps): JSX.Element => {
  const [anchorEl, setAnchorEl] = useState<HTMLElement | null>(null);

  const handlePopoverOpen = (event: React.MouseEvent<HTMLElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handlePopoverClose = () => {
    setAnchorEl(null);
  };

  const open = Boolean(anchorEl);
  const theme = useTheme();

  return (
    <div>
      <IconButton
        {...iconButtonProps}
        aria-haspopup="true"
        onMouseEnter={handlePopoverOpen}
        onMouseLeave={handlePopoverClose}
        sx={{
          color: iconButtonProps.color ? undefined : theme.palette.text.primary,
          ...iconButtonProps.sx,
        }}
      />
      <Popover
        sx={{
          pointerEvents: "none",
        }}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "center",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "center",
        }}
        onClose={handlePopoverClose}
        disableRestoreFocus
        {...popoverProps}
        open={open}
        anchorEl={anchorEl}
      />
    </div>
  );
};
