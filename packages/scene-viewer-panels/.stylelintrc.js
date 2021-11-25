module.exports = {
  customSyntax: "postcss-syntax",
  plugins: [
    "stylelint-declaration-block-no-ignored-properties",
    "stylelint-order",
  ],
  extends: [
    "stylelint-config-standard",
    "stylelint-config-recommended",
    "stylelint-config-styled-components",
    "@stylelint/postcss-css-in-js",
    "stylelint-config-prettier",
  ],
  rules: {
    "font-family-no-missing-generic-family-keyword": true,
    "declaration-block-no-shorthand-property-overrides": true,
    "declaration-block-trailing-semicolon": "always",
    "selector-pseudo-element-colon-notation": "double",
    "order/properties-alphabetical-order": true,
    "plugin/declaration-block-no-ignored-properties": true,
    // for CSS in JS
    "rule-empty-line-before": null,
    "no-empty-first-line": null,
    "no-eol-whitespace": null,
    "value-keyword-case": null,
  },
  reportNeedlessDisables: true,
  reportInvalidScopeDisables: true,
};
