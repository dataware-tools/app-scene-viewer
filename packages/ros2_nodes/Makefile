#
# Makefile
#

VERSION ?= $(shell ./utils/get-version.sh)
COMMIT_ID := $(shell git rev-parse HEAD)

IMAGE_PREFIX ?= registry.gitlab.com/dataware-tools
IMAGE_NAME ?= app-scene-viewer
IMAGE_TAG ?= $(shell basename ${PWD})-$(VERSION)
$(warning Image: $(IMAGE_PREFIX)/$(IMAGE_NAME):$(IMAGE_TAG))

help: ## Show this help
	@echo "Help"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "    \033[36m%-20s\033[93m %s\n", $$1, $$2}'

.PHONY: default
default: help

.PHONY: build-image
build-image:	## Build an image
	DOCKER_BUILDKIT=1 docker build -t $(IMAGE_PREFIX)/$(IMAGE_NAME):$(IMAGE_TAG) .

.PHONY: publish-image
publish-image:    ## Publish an image
	docker push $(IMAGE_PREFIX)/$(IMAGE_NAME):$(IMAGE_TAG)

.PHONY: run-image
run-image:    ## Run an image
	docker run -it --rm -p 9090:9090 $(IMAGE_PREFIX)/$(IMAGE_NAME):$(IMAGE_TAG)

.PHONY: build
build: build-image   ## build on image:warning
	docker run -it --rm -v ${PWD}:/opt/ros2_nodes:rw $(IMAGE_PREFIX)/$(IMAGE_NAME):$(IMAGE_TAG) bash -c "cd /opt/ros2_nodes && colcon build"
