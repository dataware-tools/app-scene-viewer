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

# Start again with a clean nginx container
FROM nginx:1-alpine

# For backwards compatibility, patch the server config to change the port
RUN sed -i 's/listen  *80;/listen 8080;/g' /etc/nginx/conf.d/default.conf
EXPOSE 8080

# Copy the build products to the web root
COPY --from=builder /app/packages/webviz/__static_webviz__ /usr/share/nginx/html