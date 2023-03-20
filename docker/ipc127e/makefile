current_dir = $(shell pwd)

download-sources:
	git clone --branch main https://github.com/EduArt-Robotik/edu_perception.git
	cd edu_perception; git submodule update --init --recursive

build-docker:
	docker build -t eduart-perception-ipc127e:0.1.0 .

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_perception
