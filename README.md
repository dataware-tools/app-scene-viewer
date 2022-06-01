# app-scene-viewer
## First things to do
**Dependencies:**
- [Node.js](https://nodejs.org/en/) v14+
- [Yarn](https://yarnpkg.com/getting-started/install)
- [Git LFS](https://git-lfs.github.com/)

**Bootstrap repository:**

1. Clone repo
1. Run `git submodule update -r -i`
1. Run `yarn install`
1. Run `yarn bootstrap`


## Develop webapp
```sh
# To launch app
$ yarn start    # start in production mode
$ yarn dev      # start in develop mode

# To launch the storybook:
$ yarn storybook:scene-viewer-panels    # launch for panels developed by HDL 
$ yarn storybook:fox-glove-studio    # launch for components built in fox-glove-studio

# To test app
$ yarn test

# To build app
$ yarn build

```

## Launch app in docker container
```sh
docker compose up
```

## Start backend server (rosbridge) for development

```bash
cd packages/ros1_nodes
docker-compose up

```