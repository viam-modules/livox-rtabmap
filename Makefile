BUILD_DIR   := build
CONFIG      := config/default.json
VIAM_CONFIG := config/viam.json
J           := $(shell nproc)
ENV_FILE    ?=

.PHONY: all build viam setup setup-viam run run-viam clean rebuild help

all: build

# ── Build ──────────────────────────────────────────────────────────────────────

build: $(BUILD_DIR)/Makefile
	cmake --build $(BUILD_DIR) -j$(J)

viam: $(BUILD_DIR)/Makefile
	cmake $(BUILD_DIR) -DWITH_VIAM=ON
	cmake --build $(BUILD_DIR) -j$(J)

$(BUILD_DIR)/Makefile:
	mkdir -p $(BUILD_DIR)
	git submodule update --init
	cmake -B $(BUILD_DIR) -S .

# ── Run ────────────────────────────────────────────────────────────────────────

run: build
	./$(BUILD_DIR)/livox_rtabmap $(CONFIG)

run-viam: viam
	$(if $(ENV_FILE),set -a && . $(ENV_FILE) && set +a &&,) ./$(BUILD_DIR)/livox_rtabmap $(VIAM_CONFIG)

# ── Setup (system deps + rtabmap from source) ──────────────────────────────────

setup:
	sudo apt install -y \
	    libpcl-dev qtbase5-dev libopenmpi-dev \
	    nlohmann-json3-dev cmake git build-essential

setup-viam:
	sudo apt install -y \
	    libabsl-dev libboost-all-dev libgrpc++-dev libprotobuf-dev \
	    libssl-dev libre2-dev protobuf-compiler-grpc \
	    libeigen3-dev ninja-build
	@# xtl (header-only, required by viam-cpp-sdk)
	cd /tmp && git clone --depth 1 --branch 0.8.1 https://github.com/xtensor-stack/xtl.git && \
	    cmake -S xtl -B xtl/build -DCMAKE_BUILD_TYPE=Release -G Ninja && \
	    sudo cmake --install xtl/build --prefix /usr/local && \
	    rm -rf /tmp/xtl
	@# xtensor (header-only, required by viam-cpp-sdk)
	cd /tmp && git clone --depth 1 --branch 0.27.1 https://github.com/xtensor-stack/xtensor.git && \
	    cmake -S xtensor -B xtensor/build -DCMAKE_BUILD_TYPE=Release -G Ninja && \
	    sudo cmake --install xtensor/build --prefix /usr/local && \
	    rm -rf /tmp/xtensor
	@if ldconfig -p 2>/dev/null | grep -q librtabmap; then \
	    echo "rtabmap already installed — skipping"; \
	else \
	    echo "Building rtabmap from source..."; \
	    git clone --depth 1 https://github.com/introlab/rtabmap.git /tmp/rtabmap; \
	    cmake -B /tmp/rtabmap/build -S /tmp/rtabmap; \
	    cmake --build /tmp/rtabmap/build -j$(J); \
	    sudo cmake --install /tmp/rtabmap/build; \
	    sudo ldconfig; \
	fi

# ── Misc ───────────────────────────────────────────────────────────────────────

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean build

help:
	@echo "Targets:"
	@echo "  make [build]    Build (standard, no Viam)"
	@echo "  make viam       Build with Viam C++ SDK support (-DWITH_VIAM=ON)"
	@echo "  make setup      Install system deps and build rtabmap from source"
	@echo "  make setup-viam Install Viam C++ SDK and its deps (run before make viam)"
	@echo "  make run        Build and run with config/default.json"
	@echo "  make run-viam   Build with Viam and run with config/viam.json"
	@echo "  make clean      Remove build directory"
	@echo "  make rebuild    clean + build"
	@echo ""
	@echo "Variables:"
	@echo "  J=<N>           Parallel jobs (default: nproc)"
	@echo "  CONFIG=<path>   Config for 'make run' (default: config/default.json)"
	@echo "  ENV_FILE=<path> Env file to source before run-viam (sets VIAM_API_KEY/ID)"
