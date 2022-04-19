export const color = {
  gray: (opacity?: number): string =>
    `rgba(247, 247, 243, ${0.1 + 0.15 * (opacity ?? 3)})`,
};
