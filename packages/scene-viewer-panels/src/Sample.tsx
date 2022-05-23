import Box from "@mui/material/Box";

export const SamplePresentation = () => {
  const color = "white";
  return (
    <>
      <Box
        sx={{
          backgroundColor: "hotpink",
          "&:hover": {
            color: color,
          },
        }}
      >
        div + emotion
      </Box>
    </>
  );
};

export const Sample = (): JSX.Element => {
  return <SamplePresentation />;
};
