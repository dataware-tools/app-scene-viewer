/** @jsx jsx */
import { css, jsx } from "@emotion/react";
import { Fragment } from "react";

export const SamplePresentation = () => {
  const color = "white";
  return (
    <Fragment>
      <div
        css={css`
          background-color: hotpink;
          &:hover {
            color: ${color};
          }
        `}
      >
        div + emotion
      </div>
    </Fragment>
  );
};

export const Sample = (): JSX.Element => {
  return <SamplePresentation />;
};
