// See https://2-hats.hateblo.jp/entry/2015/09/04/061053
export const decodeUnicode = (unicodeStr: string) => {
  return unicodeStr.replace(/(\\u)([0-9A-F]{4})/g, function (_, __, p2) {
    return String.fromCharCode(parseInt(p2, 16));
  });
};

export const sleep = (ms: number) =>
  new Promise((resolve) => setTimeout(resolve, ms));
