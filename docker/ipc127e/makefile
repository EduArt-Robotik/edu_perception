image_name = eduart-perception-ipc127e:0.1.0

download-sources:
	git clone --branch main https://github.com/EduArt-Robotik/edu_perception.git
	cd edu_perception; git submodule update --init --recursive

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_perception
