current_dir = $(shell pwd)

download-sources:
	git clone --branch main https://github.com/EduArt-Robotik/edu_perception.git

build-docker: download-sources
	docker build -t eduard-ipc127e-perception:0.1.0 .

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
