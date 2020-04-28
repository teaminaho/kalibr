.PHONY: build
build:
	sudo docker build -t kalibr .

.PHONY: run
run:
	xhost + local:root
	sudo docker run -it \
    --network="host" \
	--env=DISPLAY=$(DISPLAY) \
	--env=QT_X11_NO_MITSHM=1 \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/var/run/docker.sock:/var/run/docker.sock" \
	 kalibr /bin/bash
