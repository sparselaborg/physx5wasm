FROM emscripten/emsdk:6.0.3

RUN apt-get update \
	&& apt-get install --no-install-recommends -y ninja-build \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /src
ENTRYPOINT ["/bin/bash"]
