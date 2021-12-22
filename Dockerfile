# syntax=docker/dockerfile:1.0.0-experimental

FROM node:12.22.7 AS deps
WORKDIR /app
RUN wget -O /bin/jq https://github.com/stedolan/jq/releases/download/jq-1.6/jq-linux64 && chmod +x /bin/jq
COPY ./packages ./packages
COPY ./package.json ./package.json
COPY ./package-lock.json ./package-lock.json
COPY ./lerna.json ./lerna.json
COPY ./.husky ./.husky
COPY ./.git ./.git
RUN npm -g config set user root && npm -g config set unsafe-perm true
RUN npm run bootstrap

FROM docker:20.10.11-dind AS test
RUN apk update && apk add \
  bash \
  git \
  jq \
  nodejs \
  npm
WORKDIR /app
COPY --from=deps /app .
COPY docker-entrypoint.sh /app/docker-entrypoint.sh
ENTRYPOINT ["/app/docker-entrypoint.sh"]

# Rebuild the source code only when needed
FROM node:12.22.7-alpine AS builder
WORKDIR /app
COPY --from=deps /app .
RUN npm run build && npm run build-static-webviz

# Production image, copy all the files and run next
FROM node:12.22.7-alpine AS production
WORKDIR /app

ENV NODE_ENV production

RUN npm install -g http-server
COPY --from=builder /app/packages/webviz/__static_webviz__ /app/__static__webviz__

EXPOSE 8080

COPY docker-entrypoint.sh /app/docker-entrypoint.sh
ENTRYPOINT ["/app/docker-entrypoint.sh"]
CMD ["http-server", "/app/__static_webviz__"]