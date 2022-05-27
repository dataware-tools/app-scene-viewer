/* eslint @typescript-eslint/no-var-requires: 0 */
const esbuild = require("esbuild");
const config = require("./esbuild");

esbuild
  .build({
    ...config.CommonConfig,
    format: "cjs",
    outfile: "dist/index.js",
    watch: {
      onRebuild(error, result) {
        if (error) console.error("watch build failed:", error);
        else console.log("watch build succeeded:", result);
      },
    },
  })
  .catch(() => process.exit(1))
  .then(() => {
    console.log("watching...");
  });
