/* eslint @typescript-eslint/no-var-requires: 0 */
const esbuild = require("esbuild");
const packageInfo = require("../package.json");

const CommonConfig = {
  entryPoints: ["src/index.ts"],
  bundle: true,
  sourcemap: true,
  target: ["es6"],
  external: Object.keys(packageInfo.peerDependencies),
  loader: { ".png": "file" },
  logLevel: "info",
};

esbuild
  .build({ ...CommonConfig, format: "cjs", outfile: "dist/index.js" })
  .catch(() => process.exit(1));

module.exports = { CommonConfig };
