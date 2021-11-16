import React from "react";

export type SamplePresentationProps = SampleProps;

export type SampleProps = {};

export const SamplePresentation = (): JSX.Element => {
  return <div>test</div>;
};

export const Sample = (): JSX.Element => {
  return <SamplePresentation />;
};
