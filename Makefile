#
# Makefile to build and run dockerfile
#

IMAGE_ID ?= 4e0065656dc2
GITHUB_USRNAME ?= cmusatyalab
VERSION ?= version1.0

# Handle docker-push arguments
ifeq (docker-push,$(firstword $(MAKECMDGOALS)))
	IMAGE_ID := $(or $(word 2,$(MAKECMDGOALS)),$(IMAGE_ID))
	VERSION := $(or $(word 3,$(MAKECMDGOALS)),$(VERSION))
	GITHUB_USRNAME := $(or $(word 4,$(MAKECMDGOALS)),$(GITHUB_USRNAME))
endif

# Handle docker-git-run, docker-run, docker-build, and docker-pull arguments
ifeq ($(filter docker-git-run docker-run docker-build docker-pull,$(firstword $(MAKECMDGOALS))), $(firstword $(MAKECMDGOALS)))
	VERSION := $(or $(word 2,$(MAKECMDGOALS)),$(VERSION))
	GITHUB_USRNAME := $(or $(word 3,$(MAKECMDGOALS)),$(GITHUB_USRNAME))
endif

# Handle docker-env-build, docker-env-run, and docker-env-git-run arguments
ifeq ($(filter docker-env-build docker-env-run docker-env-git-run,$(firstword $(MAKECMDGOALS))),$(firstword $(MAKECMDGOALS)))
	GITHUB_USRNAME := $(or $(word 2,$(MAKECMDGOALS)),$(GITHUB_USRNAME))
endif

process_args:

all: process_args
	

# docker-build [version] [username] 
docker-build: process_args
	sudo docker build -t $(GITHUB_USRNAME)/openfluid:$(VERSION) -f Dockerfile .

# docker-run [version] [username] 
docker-run: process_args
	sudo docker run --gpus all --rm -it -p 9099:9099 $(GITHUB_USRNAME)/openfluid:$(VERSION)

# docker-push [image-id] [version] [username] 
docker-push: process_args
	echo $(CR_PAT) | sudo docker login ghcr.io -u $(GITHUB_USRNAME) --password-stdin && \
	sudo docker tag $(IMAGE_ID) ghcr.io/$(GITHUB_USRNAME)/openfluid:$(VERSION) && \
	sudo docker push ghcr.io/$(GITHUB_USRNAME)/openfluid:$(VERSION)

# docker-pull:
# 	echo $(CR_PAT) | sudo docker login ghcr.io -u $(GITHUB_USRNAME) --password-stdin && \
# 	sudo docker inspect ghcr.io/$(GITHUB_USRNAME)/openfluid:$(VERSION) | jq -r '.[0].Id' | \
# 	xargs -I {} sudo docker pull ghcr.io/$(GITHUB_USRNAME)/openfluid@{}

# docker-pull [version] [username] 
docker-pull: process_args
	echo $(CR_PAT) | sudo docker login ghcr.io -u $(GITHUB_USRNAME) --password-stdin
	sudo docker pull ghcr.io/$(GITHUB_USRNAME)/openfluid:$(VERSION)

# docker-git-run [version] [username] 
docker-git-run: process_args
	sudo docker run --gpus all --rm -it -p 9099:9099 ghcr.io/$(GITHUB_USRNAME)/openfluid:$(VERSION)

# docker-env-build [username] 
docker-env-build: process_args
	sudo docker build -t $(GITHUB_USRNAME)/openfluid:env -f DockerfileBuildEnv .

# docker-env-run [username] 
docker-env-run: process_args
	sudo docker run --gpus all -it -p 9099:9099 --rm --name=openfluid-env --mount type=bind,source=${PWD}/server,target=/server $(GITHUB_USRNAME)/openfluid:env

# docker-env-git-run [username] 
docker-env-git-run: process_args
	sudo docker run --gpus all -it -p 9099:9099 --rm --name=openfluid-env --mount type=bind,source=${PWD}/server,target=/server ghcr.io/$(GITHUB_USRNAME)/openfluid:env

.PHONY: all docker-env-build docker-env-run docker-build docker-run docker-pull docker-push docker-git-run