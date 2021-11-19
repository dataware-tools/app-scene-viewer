import React from "react";
import { StylesProvider } from "@material-ui/core/styles";
import "./styles/global.scss";

export const parameters = {
  actions: { argTypesRegex: "^on[A-Z].*" },
  layout: "fullscreen",
};

export const decorators = [
  (Story, context) => {
    return (
      <>
        <StylesProvider injectFirst>
          <Story {...context} />
        </StylesProvider>
      </>
    );
  },
];
