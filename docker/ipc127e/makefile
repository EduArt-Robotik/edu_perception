image_name = eduart-perception-ipc127e:0.1.0

install-udev-rule:
	sudo cp 80-movidius.rules /etc/udev/rules.d/
	sudo service udev reload
	sudo service udev restart

uninstall-udev-rule:
	sudo rm /etc/udev/rules.d/80-movidius.rules
	sudo service udev reload
	sudo service udev restart	

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
