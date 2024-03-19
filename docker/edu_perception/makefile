image_name = eduart-perception:0.3.0

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

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: download-sources build-docker
	$(info Creating EduArt's robot library Docker image "$(image_name)".)

clean:
	rm -rf edu_perception
