/* eslint @typescript-eslint/no-var-requires: 0 */
const esbuild = require("esbuild");
const packageInfo = require("../package.json");

const CommonConfig = {
  entryPoints: ["src/index.ts"],
  bundle: true,
  sourcemap: true,
  target: ["es6"],
  external: Object.keys(packageInfo.peerDependencies),
};

esbuild
  .build({ ...CommonConfig, format: "cjs", outfile: "dist/index.js" })
  .catch(() => process.exit(1));

esbuild
  .build({ ...CommonConfig, format: "esm", outfile: "dist/index.modern.js" })
  .catch(() => process.exit(1));
