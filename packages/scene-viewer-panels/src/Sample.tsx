import { css } from "@emotion/css";

export const SamplePresentation = () => {
  const color = "white";
  return (
    <>
      <div
        className={css`
          background-color: hotpink;
          &:hover {
            color: ${color};
          }
        `}
      >
        div + emotion
      </div>
    </>
  );
};

export const Sample = (): JSX.Element => {
  return <SamplePresentation />;
};
