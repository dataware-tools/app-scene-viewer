# syntax=docker/dockerfile:1.0.0-experimental

FROM node:16 AS deps
WORKDIR /app
RUN wget -O /bin/jq https://github.com/stedolan/jq/releases/download/jq-1.6/jq-linux64 && chmod +x /bin/jq
COPY ./.git ./.git
COPY ./.husky ./.husky
COPY ./.yarn ./.yarn
COPY ./.yarnrc.yml ./.yarnrc.yml
COPY ./package.json ./package.json
COPY ./yarn.lock ./yarn.lock
COPY ./packages ./packages
RUN yarn install --immutable

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
FROM node:16 AS builder
WORKDIR /app
COPY --from=deps /app .
RUN yarn run build

# Release stage
FROM caddy:2 AS serve-prod-in-local
WORKDIR /src
COPY --from=builder /app/packages/foxglove-studio/web/.webpack ./

EXPOSE 8080
CMD ["caddy", "file-server", "--listen", ":8080"]

# Start again with a clean nginx container
FROM nginx:1-alpine

# For backwards compatibility, patch the server config to change the port
RUN sed -i 's/listen  *80;/listen 8080;/g' /etc/nginx/conf.d/default.conf
EXPOSE 8080

# Copy the build products to the web root
COPY --from=builder /app/packages/foxglove-studio/web/.webpack /usr/share/nginx/html